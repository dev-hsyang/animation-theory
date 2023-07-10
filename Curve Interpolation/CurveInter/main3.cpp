//
//  main3.cpp
//  CurveInter
//
//  Created by Hyun Joon Shin on 2022/03/08.
//

#define GL_SILENCE_DEPRECATION

#include <GLFW/glfw3.h>
#include "nvg/nanovg.h"
#include "nvg/nanovg_gl.h"

int main(int argc, const char * argv[]) {

	glfwInit();
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4 );
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1 );
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, true);
	GLFWwindow* window = glfwCreateWindow(800, 600, "CurveInter", nullptr, nullptr);
	glfwMakeContextCurrent(window);
	
	
	NVGcontext* vg = NULL;


	while( !glfwWindowShouldClose(window) ) {
		glClear(GL_COLOR_BUFFER_BIT);
		glfwSwapBuffers(window);
		glfwPollEvents();
	}
	return 0;
}
