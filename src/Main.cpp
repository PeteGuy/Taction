
#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <vector>


#include <imgui.h>
#include <imgui_impl_glfw.h>



#include <imgui_impl_opengl3.h>
#include <imgui_impl_glfw.h>



#include "RigidBody.h"
#include "Render.h"
#include "Collision.h"
#include "timeOperations.h"
#include "PhysicProcess.h"

#include "Debug.h"


//On choisi un dt de 16ms (1/60fps)

std::chrono::nanoseconds accumulator(0ns);

// Vector that contains all of our physical objects
//std::vector<std::unique_ptr<RigidBody>> bodies;




void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
	//if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS)
	
	double mouseX;
	double mouseY;

	if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS)
	{
		glfwGetCursorPos(window, &mouseX, &mouseY);
		//bodies.push_back(std::make_unique<RigidBody>(new CircleShape(40.0f), mouseX, mouseY));
		PhysicProcess::bodies.push_back(std::make_unique<RigidBody>(new OBBCollider(40.0f,40.0f), mouseX, mouseY));
		//PhysicProcess::bodies.back()->transform.velocity = glm::vec2(0.05f,-10.0f);
		
	}
	else if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS)
	{
		glfwGetCursorPos(window, &mouseX, &mouseY);
		PhysicProcess::bodies.push_back(std::make_unique<RigidBody>(new CircleCollider(40.0f), mouseX, mouseY));
		//PhysicProcess::bodies.push_back(std::make_unique<RigidBody>(new AABB_Shape(40.0f,40.0f), mouseX, mouseY));
		//PhysicProcess::bodies.back()->transform.velocity = glm::vec2(0.05f, 10.0f);
		
	}
}


int main()
{
	

	


	
	std::cout.setf(std::ios::fixed);

	glm::vec2 v(1, 2);
	glm::mat2 r(0,-1,1,0);

	v = r*v;

	/*std::cout << v.x << "\n";
	std::cout << v.y << "\n";*/

	

	
	
	
	Render::initialize();
	Debug::initialize();
	
	GLFWwindow* window = Render::renderWindow;
	Shader defaultShader = Render::defaultShader;

	Primitive testP(PrimitiveType::Rectangle);
	/*// Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;
	//io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
	//io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

	// Setup Dear ImGui style
	ImGui::StyleColorsDark();
	//ImGui::StyleColorsClassic();

	// Setup Platform/Renderer backends
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	//const char* glslVersion = (char*)glGetString(GL_SHADING_LANGUAGE_VERSION);
	const char* version = "#version 130";
	//std::cout << "the returned version is : " << (char*)glad_glGetString(GL_VERSION) << "\n";
	ImGui_ImplOpenGL3_Init(version);
	//ImGui_ImplOpenGL3_Init((char*)glad_glGetString(GL_SHADING_LANGUAGE_VERSION));
	//ImGui_ImplOpenGL3_Init("#130");
	//Todo*/
	

	
	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
	glPointSize(2);

	

	glfwSetMouseButtonCallback(window, mouse_button_callback);
	
	using clock = std::chrono::high_resolution_clock;
	auto frameStart = std::chrono::high_resolution_clock::now();
	auto currentTime = std::chrono::high_resolution_clock::now();
	
	float tp1[] =
	{
		0.0f,0.0f,
		0.0f,40.0f,
		40.0f,40.0f,
		
	};

	float tp2[] =
	{
		0.0f,0.0f,
		40.0f,40.0f,
		80.0f,20.0f,
		80.0f,-12.0f,
		37.0f,-70.0f,
		-30.0f,-70.0f,

	};

	float tp3[] =
	{
		0.0f,0.0f,
		40.0f,40.0f,
		80.0f,20.0f,
		80.0f,-12.0f,

	};

	//RigidBody testPolygon1(new PolygonShape(tp1, 3), 400, 300);
	RigidBody testPolygon3(new PolygonCollider(tp3, 4), 400, 300);
	RigidBody testPolygon2(new PolygonCollider(tp2, 6), 400, 300);

	testPolygon2.update();
	testPolygon3.update();

	RigidBody testOBB(new OBBCollider(50.0f,50.0f),100,100);
	RigidBody testOBB2(new OBBCollider(50.0f, 50.0f), 50, 50);
	//testOBB2.angle = glm::pi<float>()/4;

	testOBB.update();
	testOBB2.update();


	/*for (int i = 0; i < testPolygon3.shape->vecPoints.size(); i++)
	{
		std::cout << testPolygon3.shape->vecPoints[i].x<<"\n";
	}
	std::cout << testPolygon3.shape->vecPoints.size() << "\n";*/


	// Bottom plateform of infinite mass to represent the ground
	PhysicProcess::bodies.push_back(std::make_unique<RigidBody>(new OBBCollider(700.0f, 50.0f), 400, 600));
	PhysicProcess::bodies[0]->inv_mass = 0;
	PhysicProcess::bodies[0]->mass = 0;


	
	//PhysicProcess::bodies.push_back(std::make_unique<RigidBody>(new OBBCollider(50.0f, 50.0f), 0, 0));
	
	//defaultShader.makeCurrentShader();
	
	//glLineWidth(10);

	double mouseX;
	double mouseY;
	// Main while loop
	

	ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

	while (!glfwWindowShouldClose(window))
	{
		testPolygon2.angle += 2;
		glfwGetCursorPos(window, &mouseX, &mouseY);

		//testPolygon3.position = glm::vec2(mouseX,mouseY);
		//PhysicProcess::bodies[1]->position = glm::vec2(mouseX, mouseY);
		testOBB2.position = glm::vec2(mouseX, mouseY);
		currentTime = std::chrono::high_resolution_clock::now();
		// Store the time elapsed since the last frame began
		accumulator += currentTime - frameStart;

		// Record the starting of this frame
		frameStart = currentTime;


		if (accumulator > std::chrono::microseconds(200ms))
		{
			accumulator = std::chrono::microseconds(200ms);
		}

		while (accumulator > dt)
		{
			PhysicProcess::applyGravity();
			PhysicProcess::BroadPhase();
			PhysicProcess::NarrowPhase();
			
			//update of our bodies
			for (auto ita = PhysicProcess::bodies.begin(); ita != PhysicProcess::bodies.end(); ita++)
			{
				(*ita)->update();
			}
			
			
			accumulator -= dt;
		}

		//We calculate the alpha for possible interpolation (not implemented yet)
		auto alpha = (float)accumulator.count() / dt.count();



		// Render part
		// Specify the color of the background
		glClearColor(0.07f, 0.13f, 0.17f, 1.0f);
		// Clean the back buffer and assign the new color to it
		glClear(GL_COLOR_BUFFER_BIT);

		//Render our bodies here
		/*Render::renderBody(&cTest);
		Render::renderBody(&cTest2);
		Render::renderBody(&rTest);
		Render::renderBody(&test);
		Render::renderBody(&test2);*/

		for (auto ita = PhysicProcess::bodies.begin(); ita != PhysicProcess::bodies.end(); ita++)
		{
			Render::renderBody(ita->get());
		}
		/*Render::renderBody(&testOBB);
		Render::renderBody(&testOBB2);*/
		//Render::drawPrimitive(&testP, glm::vec2(400,300),0 ,50,25);
		//Debug::drawDebugLine({ 400.0f,20.0f }, { 400.0f, 300.0f });
	//})
		//Render::renderBody(&testPolygon1);
		
		/*
		// Start the Dear ImGui frame
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		static float f = 0.0f;
		static int counter = 0;

		ImGui::Begin("Hello, world!");                          // Create a window called "Hello, world!" and append into it.

		ImGui::Text("This is some useful text.");               // Display some text (you can use a format strings too)
		//ImGui::Checkbox("Demo Window", &show_demo_window);      // Edit bools storing our window open/close state
		//ImGui::Checkbox("Another Window", &show_another_window);

		ImGui::SliderFloat("float", &f, 0.0f, 1.0f);            // Edit 1 float using a slider from 0.0f to 1.0f
		ImGui::ColorEdit3("clear color", (float*)&clear_color); // Edit 3 floats representing a color

		if (ImGui::Button("Button"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
			counter++;
		ImGui::SameLine();
		ImGui::Text("counter = %d", counter);

		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		ImGui::End();

		
		ImGui::Render();
		int display_w, display_h;
		

		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData()); */

		
		/*Render::renderBody(&testPolygon2);
		Render::renderBody(&testPolygon3);
		testPolygon2.update();
		testPolygon3.update();*/

		
		if (PolygonVSPolygonSAT(&testPolygon2, &testPolygon3))
		{
			//std::cout << "oezi\n";
		}
		else
		{
			//std::cout << "fzefezfe\n";
		}
		
		
		
		// Swap the back buffer with the front buffer
		glfwSwapBuffers(window);
		// Take care of all GLFW events
		glfwPollEvents();
	}

	//Shutdown
	
	/*ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();*/

	Render::terminate();
	
	return 0;
}


