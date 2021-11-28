#pragma once
#include "Macros.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

namespace PhysicData
{
	struct MassData
	{
		float mass;
		float inv_mass;

		// For rotations (not covered in this article)
		float inertia;
		float inv_inert;
	};

	struct MaterialData
	{
		float density;
		float restitution;

		float staticFriction;
		float dynamicFriction;
	};

	struct TransformData
	{
		glm::vec2 position = glm::vec2(0.0f,0.0f);
		glm::vec2 velocity = glm::vec2(0.0f, 0.0f);;

		float angle = 0;
		float angularVelocity = 0;
	};

	float calculateSafeInverse(float mass);
	int getSupportPoint(std::vector<glm::vec2>& points, glm::vec2 axis);
	

	extern MassData defaultMassData;
	extern MaterialData defaultMatDat;
}
