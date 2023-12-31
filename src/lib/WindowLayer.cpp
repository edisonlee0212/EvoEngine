#include "WindowLayer.hpp"
#include "Graphics.hpp"
#include "Application.hpp"
#include "ProjectManager.hpp"

using namespace EvoEngine;

void WindowLayer::FramebufferSizeCallback(GLFWwindow* window, int width, int height)
{
	const auto windowLayer = Application::GetLayer<WindowLayer>();
	if (windowLayer->m_window == window)
	{
		windowLayer->m_windowSize = { width, height };
	}
	if(const auto& graphicsLayer = Application::GetLayer<Graphics>())
	{
		graphicsLayer->NotifyRecreateSwapChain();
	}

}

void WindowLayer::SetMonitorCallback(GLFWmonitor* monitor, int event)
{
	const auto windowLayer = Application::GetLayer<WindowLayer>();
	if (event == GLFW_CONNECTED)
	{
		// The monitor was connected
		for (const auto& i : windowLayer->m_monitors)
			if (i == monitor)
				return;
		windowLayer->m_monitors.push_back(monitor);
	}
	else if (event == GLFW_DISCONNECTED)
	{
		// The monitor was disconnected
		for (auto i = 0; i < windowLayer->m_monitors.size(); i++)
		{
			if (monitor == windowLayer->m_monitors[i])
			{
				windowLayer->m_monitors.erase(windowLayer->m_monitors.begin() + i);
			}
		}
	}
	windowLayer->m_primaryMonitor = glfwGetPrimaryMonitor();

}

void WindowLayer::WindowFocusCallback(GLFWwindow* window, int focused)
{
	const auto windowLayer = Application::GetLayer<WindowLayer>();
	
	if (focused)
	{
		ProjectManager::ScanProject();
	}
}

void WindowLayer::OnCreate()
{

}

void WindowLayer::OnDestroy()
{
#pragma region Windows
	glfwDestroyWindow(m_window);
	glfwTerminate();
#pragma endregion
}

GLFWwindow* WindowLayer::GetGlfwWindow() const
{
	return m_window;
}

void WindowLayer::ResizeWindow(int x, int y) const
{
	glfwSetWindowSize(m_window, x, y);
}