#include "PhysicData.h"
#include "Debug.h"


namespace PhysicData
{
	MassData defaultMassData
	{
		20.0f,
		calculateSafeInverse(20.0f),
		0.0f,
		calculateSafeInverse(20.0f)
	};



	MaterialData defaultMatDat
	{
		20.0f,
		0.9f,
		0.1f,
		0.05f
	};

	float calculateSafeInverse(float mass)
	{
		return (mass != 0) ? 1 / mass : 0;
	}

	int getSupportPoint(std::vector<glm::vec2>& points, glm::vec2 axis)
	{
		float bestProjection = -FLT_MAX;
		int bestIndex = -1;

		float currentProjection;
		
		for (int i = 0; i < points.size(); i++)
		{
			currentProjection = glm::dot(points[i], axis);
			if (currentProjection > bestProjection)
			{
				bestIndex = i;
				bestProjection = currentProjection;
			}
			//Debug::log("current", currentProjection);
			
		}
		
		return bestIndex;

	}

}
