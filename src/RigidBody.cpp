#include "RigidBody.h"
#include "timeOperations.h"
#include "Debug.h"
//#include <glm/detail/func_geometric.inl>


RigidBody::RigidBody()
{
	// We use default values for now

	collider = new OBBCollider(40.0f, 40.0f);

	//Mass info
	mass = 20.0f;
	inv_mass = PhysicData::calculateSafeInverse(mass);
	inertia = 20.0f;
	inv_inert = PhysicData::calculateSafeInverse(inertia);

	//Material info
	density = 20.0f;
	restitution = 0.9f;
	staticFriction = 0.1f;
	dynamicFriction = 0.05f;

	//Transform info
	position = glm::vec2(0, 0);
	velocity = glm::vec2(0.0f, 0.0f);;
	angle = 0.0f;
	angularVelocity = 0.0f;

	force = glm::vec2(0.0f, 0.0f);
	layer = 2;


	

	this->collider = collider;
	collider->setPoints();

	

	
	
}

RigidBody::RigidBody(Collider* collider,float x,float y,float angle, float mass, float inertia, float density, float restitution, float staticFriction, float dynamicFriction, uint32_t layer)
{
	
	
	// We use default values for now
	
	//Mass info
	this->mass = mass;
	inv_mass = PhysicData::calculateSafeInverse(mass);
	this->inertia = inertia;
	inv_inert = PhysicData::calculateSafeInverse(inertia);

	//Material info
	this->density = density;
	this->restitution = restitution;
	this->staticFriction = staticFriction;
	this->dynamicFriction = dynamicFriction;

	this->layer = layer;

	//Transform info
	this->position = glm::vec2(x, y);
	this->velocity = glm::vec2(0.0f, 0.0f);;
	this->angle = 0.0f;
	this->angularVelocity = 0.0f;

	this->force = glm::vec2(0.0f, 0.0f);
	
	this->angle = angle;
	rotationMat = glm::mat2(cos(angle), sin(angle), -sin(angle), cos(angle));


	this->collider = collider;
	collider->setPoints();


	

}






void RigidBody::render()
{
	// body render is currently fully handled by the render namespace (to reconsider)
	

}


void RigidBody::update()
{

	
	velocity += (force * inv_mass)*dtValue;
	
	position += velocity * dtValue;
	angle += angularVelocity * dtValue;
	
	rotationMat = glm::mat2(cos(angle),sin(angle),-sin(angle),cos(angle));
	force = glm::vec2(0.0f,0.0f);
	angularVelocity /= 1.12f;
}

//Returns light AAB which allows for easy calculation during broad phase
AABB RigidBody::getDefaultAABB()
{
	return collider->getAABB(position, rotationMat);
}

void RigidBody::applyForce(glm::vec2 force)
{
	this->force += force;
}

void RigidBody::applyImpulse(const glm::vec2& impulse, const glm::vec2& contactPoint)
{
	velocity += inv_mass * impulse;
	angularVelocity += inv_inert * crossVec2(contactPoint,impulse);
	Debug::log("point", contactPoint);
	
	
}

