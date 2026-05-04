
#pragma once
#include <iostream>

namespace evo_engine {

/**
 * \class Console
 * \brief A final class for logging different levels of console messages (log, error, warning).
 */
class Console final {
 public:
  static Console& GetInstance();

 private:
 public:
  /**
   * \brief Logs an informational message to the console.
   *
   * This function is used to output a general log message to the console for debugging or tracking purposes.
   * Example usage:
   * \code
   * evo_engine::Console::Log("This is a log message.");
   * \endcode
   *
   * \param msg The message to log in string format.
   */
  static void Log(const std::string& msg);

  /**
   * \brief Logs an error message to the console.
   *
   * This function is used to output an error message, which could signify issues requiring attention.
   * Example usage:
   * \code
   * evo_engine::Console::Error("An error occurred.");
   * \endcode
   *
   * \param msg The error message to log in string format.
   */
  static void Error(const std::string& msg);

  /**
   * \brief Logs a warning message to the console.
   *
   * This function is used to output warning messages, indicating potential issues or considerations.
   * Example usage:
   * \code
   * evo_engine::Console::Warning("This is a warning message.");
   * \endcode
   *
   * \param msg The warning message to log in string format.
   */
  static void Warning(const std::string& msg);
};

}  // namespace evo_engine

/**
 * \brief A thread-safe logging macro for outputting informational messages.
 *
 * This macro formats the message, appends file and line details, and logs it to the console.
 *
 * Example:
 * \code
 * EVOENGINE_LOG("Hello from evo_engine!");
 * \endcode
 *
 * \param msg The informational message to be logged.
 */
#define EVOENGINE_LOG(msg)                                                              \
  {                                                                                     \
    std::stringstream ss;                                                               \
    ss << msg;                                                                          \
    evo_engine::Console::Log(ss.str());                                                 \
    std::cout << "[EvoEngine]Log: " << msg << " (" << __FILE__ << ": line " << __LINE__ \
              << ")\n==========" << std::endl;                                          \
  }

/**
 * \brief A thread-safe macro for logging error messages.
 *
 * This macro formats the error message, appends file and line details, and outputs it using the error log mechanism.
 *
 * Example:
 * \code
 * EVOENGINE_ERROR("An unexpected error occurred.");
 * \endcode
 *
 * \param msg The error message to be logged.
 */
#define EVOENGINE_ERROR(msg)                                                              \
  {                                                                                       \
    std::stringstream ss;                                                                 \
    ss << msg;                                                                            \
    evo_engine::Console::Error(ss.str());                                                 \
    std::cerr << "[EvoEngine]Error: " << msg << " (" << __FILE__ << ": line " << __LINE__ \
              << ")\n==========" << std::endl;                                            \
  }

/**
 * \brief A thread-safe macro for logging warning messages.
 *
 * This macro formats the warning message, appends file and line details, and outputs it using the warning log
 * mechanism.
 *
 * Example:
 * \code
 * EVOENGINE_WARNING("This is a warning message.");
 * \endcode
 *
 * \param msg The warning message to be logged.
 */
#define EVOENGINE_WARNING(msg)                                                              \
  {                                                                                         \
    std::stringstream ss;                                                                   \
    ss << msg;                                                                              \
    evo_engine::Console::Warning(ss.str());                                                 \
    std::cout << "[EvoEngine]Warning: " << msg << " (" << __FILE__ << ": line " << __LINE__ \
              << ")\n==========" << std::endl;                                              \
  }
