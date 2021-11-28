#pragma once
#include<glad/glad.h>
#include <vector>
#include "Macros.h"
enum class ColliderType  { Circle, OBB,AABB,Polygon,last };

struct AABB
{
	glm::vec2 min;
	glm::vec2 max;
};


class Collider
{
public:
	int nbPoints;
	//std::vector<float> points;
	std::vector<glm::vec2> vecPoints;
	// Careful, uint only goes up to 255
	//std::vector<GLuint> indices;

	ColliderType type;

	Collider();
	virtual void setPoints() = 0;
	virtual AABB getAABB(glm::vec2 position, glm::mat2 rotationMat) = 0;

	float radius;
	float halfExtentX;
	float halfExtentY;

	
};

class CircleCollider : public Collider
{
public:
	CircleCollider(float radius);
	//float radius;
	void setPoints() override;
	AABB getAABB(glm::vec2 position, glm::mat2 rotationMat) override;

};

//class AABB_Collider : public Collider
//{
//public:
//	AABB_Collider(float width, float height);
//	
//	void setPoints() override;
//	AABB getAABB(glm::vec2 position, glm::mat2 rotationMat) override;
//
//
//};


class PolygonCollider : public Collider
{
public:
	PolygonCollider(float vertices[], int nbPoints);
	void setPoints() override;
	AABB getAABB(glm::vec2 position, glm::mat2 rotationMat) override;

	void setNormals();
	void calculateCOM();

	std::vector<glm::vec2> normals;
	glm::vec2 com;
	
};


class OBBCollider : public Collider
{
public: 
	OBBCollider(float width,float height);
	AABB getAABB(glm::vec2 position, glm::mat2 rotationMat) override;
	void setPoints() override;

};

