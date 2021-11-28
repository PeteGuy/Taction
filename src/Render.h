



#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cerrno>
#include "Shader.h"
#include "RigidBody.h"
#include "Primitive.h"
#include "Macros.h"
#include "Primitive.h"


namespace Render
{
	extern float windowWidth;
	extern float windowHeight;
	extern Shader defaultShader;
	extern glm::mat4 projection;
	extern std::vector<std::unique_ptr<Primitive>> primitives;
	//Default shader

	extern GLFWwindow* renderWindow;

	void initialize();
	void terminate();
	void changeWindowSize(float width, float height);

	void renderBody(RigidBody* body);
	//Render function that uses interpolation not in use for now
	void renderBody(float alpha);
	void drawPrimitive(Primitive* primitive,glm::vec2 position = glm::vec2(),float angle = 0,float xScale = 1, float yScale = 1);

	void setNewMouseButtonCallBack(GLFWmousebuttonfun callback);
	
}



