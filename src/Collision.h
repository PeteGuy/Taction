#pragma once
#include "RigidBody.h"

struct Manifold
{
	RigidBody* A;
	RigidBody* B;
	float penetration;
	glm::vec2 normal;
	glm::vec2 contacts[2];     // Points of contact during collision
	uint32_t contact_count;
};




bool PolygonVSPolygonSAT(RigidBody *A, RigidBody *B);
bool OBBCollisionTest(RigidBody* A, RigidBody* B);


// This function serves in the broad phase
bool quickAABBvsAABB(AABB& a, AABB& b);

typedef bool (*CollisionFunction)(Manifold* m);

extern CollisionFunction collisionFunctions[(int)ColliderType::last][(int)ColliderType::last];

//Collision resolution function
bool resolveCollisionCircleCircle(Manifold* m);
bool resolveCollisionCircle_OBB(Manifold* m);
bool resolveCollisionOBB_OBB(Manifold* m);
bool resolveCollisionOBB_Circle(Manifold* m);

bool resolveCollisionPolygonCircle(Manifold* m);
bool resolveCollisionPolygonPolygon(Manifold* m);


// Execute necessary action : changing the velocity of colliding objects to separate them
void actOnCollision(Manifold* m);
void positionalCorrection(Manifold* m);