/*

 * SPDX-FileCopyrightText: Copyright (c) 2021 - 2024  NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace optix {
namespace CompileWithTasks {

inline void check( OptixResult res, const char* call, const char* file, unsigned int line )
{
    if( res != OPTIX_SUCCESS )
    {
        std::stringstream s;
        s << "Optix call in " << file << ", line " << line << " (" << call << ") failed with code " << res;
        throw std::runtime_error( s.str() );
    }
}
#define COMPILE_WITH_TASKS_CHECK( call ) check( call, #call, __FILE__, __LINE__ )


// Using the specified number of threads, execute the functions added to the work
// queue. Work can be added asynchronously by any other thread. Call terminate() to exit
// all the threads. The thread pool can be started up again after being
// terminated. Threads are started, but suspended until there is work in the queue.
struct ThreadPool
{
    std::vector<std::thread> m_pool;
    std::mutex               m_queueMutex;
    using FunctionType = std::function<void()>;
    std::deque<FunctionType> m_workQueue;
    bool                     m_killPool = false;
    std::condition_variable  m_condition;

    void startPool( int numThreads )
    {
        for( int i = 0; i < numThreads; ++i )
            m_pool.emplace_back( std::bind( &ThreadPool::workerExecute, this ) );
    }

    void addWork( FunctionType&& function )
    {
        std::lock_guard<std::mutex> lock( m_queueMutex );
        m_workQueue.push_back( function );
        // Wake up one thread to handle this new job if it's not already awake.
        m_condition.notify_one();
    }

    void workerExecute()
    {
        while( true )
        {
            FunctionType work;
            {
                std::unique_lock<std::mutex> lock( m_queueMutex );

                // Sit here and wait until there's some work to do or we are terminating the pool
                m_condition.wait( lock, [this] { return !m_workQueue.empty() || m_killPool; } );
                if( m_killPool )
                    break;
                work = m_workQueue.front();
                m_workQueue.pop_front();
            }
            work();
        }
    }

    void terminate()
    {
        {
            std::unique_lock<std::mutex> lock( m_queueMutex );
            m_killPool = true;
            // This bit of code is optional depending on whether you want to be able to
            // terminate a non-empty queue.
            if( !m_workQueue.empty() )
                throw std::runtime_error( "pool not empty" );
        }
        // Wake all threads, so they can exit the work/wait loop.
        m_condition.notify_all();
        for( size_t i = 0; i < m_pool.size(); ++i )
            m_pool[i].join();
        m_pool.clear();
    }
};

// Compiles one or more OptixModules using multiple threads contained in m_threadPool. As new
// tasks are generated from calling optixTaskExecute, add more work to the thread pool.
struct OptixTaskExecutePool
{
    ThreadPool              m_threadPool;
    unsigned int            m_maxNumAdditionalTasks;
    bool                    m_stop = false;
    std::condition_variable m_cv;

    void executeTask( OptixTask task, std::condition_variable& cv )
    {
        // When we execute the task, OptiX can generate upto the number of additional
        // tasks that we provide. [0..m_maxNumAdditionalTasks] are valid values for
        // numAdditionalTasksCreated.
        std::vector<OptixTask> additionalTasks( m_maxNumAdditionalTasks );
        unsigned int           numAdditionalTasksCreated;
        COMPILE_WITH_TASKS_CHECK( optixTaskExecute( task, additionalTasks.data(), m_maxNumAdditionalTasks, &numAdditionalTasksCreated ) );
        for( unsigned int i = 0; i < numAdditionalTasksCreated; ++i )
        {
            // Capture additionalTasks[i] by value since it will go out of scope.
            OptixTask task = additionalTasks[i];
            m_threadPool.addWork( [task, &cv, this]() { executeTask( task, cv ); } );
        }
        // Notify the thread calling executeTaskAndWait that a task has finished
        // executing.
        cv.notify_all();
    }

    // Add a module compilation task to the work queue.
    void addTaskAndExecute( OptixTask task )
    {
        m_threadPool.addWork( [task, this]() { executeTask( task, m_cv ); } );
    }

    // Monitor the work queue and wait until the compile tasks for all of the
    // OptixModules have completed.
    OptixResult waitForModuleTasks( std::vector<OptixModule>& modules )
    {
        std::mutex                   mutex;
        std::unique_lock<std::mutex> lock( mutex );
        OptixResult                  result = OPTIX_SUCCESS;
        for( OptixModule& module : modules )
        {
            OptixModuleCompileState state;
            m_cv.wait( lock, [&] {
                COMPILE_WITH_TASKS_CHECK( optixModuleGetCompilationState( module, &state ) );
                return state == OPTIX_MODULE_COMPILE_STATE_FAILED || state == OPTIX_MODULE_COMPILE_STATE_COMPLETED || m_stop;
            } );
            result = state == OPTIX_MODULE_COMPILE_STATE_FAILED || state == OPTIX_MODULE_COMPILE_STATE_IMPENDING_FAILURE ?
                         OPTIX_ERROR_UNKNOWN :
                         result;
        }
        return result;
    }

    // Wait until the task for the OptiXModule has completed.
    OptixResult waitForModuleTask( OptixModule module )
    {
        std::vector<OptixModule> modules( 1, module );
        return waitForModuleTasks( modules );
    }
};

}  // end namespace CompileWithTasks
}  // end namespace optix
