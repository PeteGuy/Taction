#include "Colliders.h"
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <iostream>

Collider::Collider()
{
	
}

/*CircleCollider* Collider::getCircleCollider()
{
	return NULL;
}
AABB_Collider* Collider::getAABB_Collider()
{
	return NULL;
}
PolygonCollider* Collider::getPolygonCollider()
{
	return NULL;
}*/


CircleCollider::CircleCollider(float radius) : Collider()
{
	// Default number of points to draw the circle (60 on the circumference and 1 for the radius line and another one to draw the final line) 
	//nbPoints = 62; 
	this->radius = radius;
	type = ColliderType::Circle;
}

void CircleCollider::setPoints()
{
	

	

	//this->vecPoints.push_back(glm::vec2(0, 0));


	////Reversed the ordre but it doesn't floatly matter
	//for (int i = nbPoints - 2; i >= 0; i--)
	//{

	//	float t = i * (2 * glm::pi<float>()) / (nbPoints - 2);
	//	

	//	
	//	this->vecPoints.push_back(glm::vec2(radius * glm::cos(t), radius * glm::sin(t)));
	//	//this->indices.push_back(nbPoints - i - 2);


	//}
	

	//this->indices.push_back(1);






}

AABB CircleCollider::getAABB(glm::vec2 position, glm::mat2 rotationMat)
{
	glm::vec2 radiusVector = radius * glm::vec2(1, 1);
	return AABB{position-radiusVector,position+radiusVector};
}


//AABB_Collider::AABB_Collider(float width, float height)
//{
//	nbPoints = 4;
//	this->width = width;
//	this->height = height;
//	type = ColliderType::AABB;
//}
//
//
//
//void AABB_Collider::setPoints()
//{
//	
//	this->vecPoints.push_back(glm::vec2(-width / 2, -height / 2));
//	
//
//	this->vecPoints.push_back(glm::vec2(-width / 2, height / 2));
//	
//
//	this->vecPoints.push_back(glm::vec2(width / 2, height / 2));
//	
//
//	this->vecPoints.push_back(glm::vec2(width / 2, -height / 2));
//	
//
//
//
//	/*this->indices.push_back(0);
//	
//	for (int i = 0; i < 4; i++)
//	{
//		this->indices.push_back(3-i);
//	
//	}*/
//	
//}
//
//AABB AABB_Collider::getAABB(glm::vec2 position, glm::mat2 rotationMat)
//{
//	glm::vec2 v = glm::vec2(width / 2, height / 2);
//	return AABB{ position - v,position + v };
//}


PolygonCollider::PolygonCollider(float vertices[],int nbPoints)
{
	// the vertices array has to be a serie of 2 dimensional vectors (no z axis)
	assert(nbPoints >= 3);// We want at least three points to form a polygone
	this->nbPoints = nbPoints; // we have to add one to render it fully 
	//int testarr[] = { 1,2,3,4,5 };
	//this->indices.push_back(0);

	/*for (int i = 0; i < nbPoints; i++)
	{
		this->indices.push_back(nbPoints-1 - i);

	}*/
	//this->points = std::vector<float>(vertices,vertices+(nbPoints*2));
	for (int i = 0; i < nbPoints; i++)
	{
		this->vecPoints.push_back(glm::vec2(vertices[i*2], vertices[(i*2) + 1]));
	}
	


	
	type = ColliderType::Polygon;
	
	calculateCOM();
	setPoints();
	
	setNormals();
	//points
}

void PolygonCollider::setPoints()
{
	
	//This method simply translate every point in order to have the origin be the center of mass
	for (auto it = vecPoints.begin(); it < vecPoints.end(); it ++)
	{
		*it = *it - com;
		
		//*it = *it - com.x;
		//*(it + 1) = *(it + 1) - com.y;
	}

	
	com = glm::vec2();
	

}


AABB PolygonCollider::getAABB(glm::vec2 position, glm::mat2 rotationMat)
{
	float minX = FLT_MAX;
	float maxX = -FLT_MAX;
	float minY = FLT_MAX;
	float maxY = -FLT_MAX;
	glm::vec2 currentPoint;
	for (auto it = vecPoints.begin(); it != vecPoints.end(); it++)
	{
		currentPoint = rotationMat * glm::vec2(it->x, it->y) + position;
		if (currentPoint.x > maxX)
		{
			maxX = currentPoint.x;
		}
		if (currentPoint.x < minX)
		{
			minX = currentPoint.x;
		}
		if (currentPoint.y > maxY)
		{
			maxY = currentPoint.y;
		}
		if (currentPoint.y < minY)
		{
			minY = currentPoint.y;
		}
		//aPoints.push_back(A->rotationMat * aShape->vecPoints[i] + A->position);

	}


	return AABB{ glm::vec2(minX,minY),glm::vec2(maxX,maxY) };
}

void PolygonCollider::setNormals()
{
	/*for (auto it = points.begin(); it != points.end()-3; it+=3)
	{												// x values    
		normals.push_back(glm::normalize(glm::vec2(*(it + 3 + 1) - *(it + 1), -(*(it + 3) - *it))));
	}*/
}


void PolygonCollider::calculateCOM()
{
	com = glm::vec2();
	//we want everypoint but one of them is repeated
	for (auto it = vecPoints.begin(); it < vecPoints.end(); it++)
	{
		
		com += *it;
		

	}

	com /= nbPoints;
	//std::cout << "COM is at x = " << com.x << "," << com.y << "\n";
}


OBBCollider::OBBCollider(float width, float height)
{
	nbPoints = 4;
	this->halfExtentX = width/2;
	this->halfExtentY = height/2;
	type = ColliderType::OBB;
}
void OBBCollider::setPoints()
{
	this->vecPoints.push_back(glm::vec2(-halfExtentX, -halfExtentY));


	this->vecPoints.push_back(glm::vec2(-halfExtentX, halfExtentY));


	this->vecPoints.push_back(glm::vec2(halfExtentX, halfExtentY));


	this->vecPoints.push_back(glm::vec2(halfExtentX, -halfExtentY));
}


AABB OBBCollider::getAABB(glm::vec2 position, glm::mat2 rotationMat)
{
	float minX = FLT_MAX;
	float maxX = -FLT_MAX;
	float minY = FLT_MAX;
	float maxY = -FLT_MAX;
	glm::vec2 currentPoint;
	for (auto it = vecPoints.begin(); it != vecPoints.end(); it++)
	{
		currentPoint = rotationMat * glm::vec2(it->x, it->y) + position;
		if (currentPoint.x > maxX)
		{
			maxX = currentPoint.x;
		}
		if (currentPoint.x < minX)
		{
			minX = currentPoint.x;
		}
		if (currentPoint.y > maxY)
		{
			maxY = currentPoint.y;
		}
		if (currentPoint.y < minY)
		{
			minY = currentPoint.y;
		}
		//aPoints.push_back(A->rotationMat * aShape->vecPoints[i] + A->position);

	}


	return AABB{ glm::vec2(minX,minY),glm::vec2(maxX,maxY) };
}

