#include "Collision.h"
#include "Debug.h"


void findAxisOfLeastPenetrationOBB(glm::vec2& xAxis, glm::vec2& yAxis, std::vector<glm::vec2>& points, float* bestDistance, int* bestIndice);
bool quickAABBvsAABB(AABB& a, AABB& b)
{
	// Exit with no intersection if found separated along an axis
	if (a.max.x < b.min.x || a.min.x > b.max.x)
		return false;
	if (a.max.y < b.min.y || a.min.y > b.max.y)
		return false;

	// No separating axis found, therefor there is at least one overlapping axis
	return true;
}


//Fonctions de résolution de collisions

CollisionFunction collisionFunctions[(int)ColliderType::last][(int)ColliderType::last] =
{
  {
	resolveCollisionCircleCircle, resolveCollisionCircle_OBB
  },
  {
	resolveCollisionOBB_Circle, resolveCollisionOBB_OBB
  },
};




bool resolveCollisionCircleCircle(Manifold* m)
{
	// Setup a couple pointers to each object
	RigidBody* A = m->A;
	RigidBody* B = m->B;

	CircleCollider* aCollider = dynamic_cast<CircleCollider*>(A->collider);
	CircleCollider* bCollider = dynamic_cast<CircleCollider*>(B->collider);

	// Vector from A to B
	glm::vec2 n = B->position - A->position;

	//Sum of the two radius
	float r = aCollider->radius + bCollider->radius;

	//No collision
	if (glm::length2(n) > r * r)
	{
		return false;
	}


	// Circles have collided, now compute manifold
	float d = glm::length(n); // perform actual sqrt

	// If distance between circles is not zero
	if (d != 0)
	{
		// Distance is difference between radius and distance
		m->penetration = r - d;

		// Utilize our d since we performed sqrt on it already within Length( )
		// Points from A to B, and is a unit vector
		m->normal = n * (1 / d);

		return true;
	}

	// Circles are on same position
	else
	{
		// Choose random (but consistent) values
		m->penetration = bCollider->radius;
		m->normal = glm::vec2(1, 0);
		return true;
	}
}





bool resolveCollisionOBB_OBB(Manifold* m)
{
	RigidBody* A = m->A;
	RigidBody* B = m->B;
	auto aCollider = dynamic_cast<OBBCollider*>(A->collider);
	auto bCollider = dynamic_cast<OBBCollider*>(B->collider);

	int aSize = aCollider->nbPoints;
	int bSize = bCollider->nbPoints;
	std::vector<glm::vec2> aPoints;
	std::vector<glm::vec2> bPoints;

	for (int i = 0; i < aSize; i++)
	{
		aPoints.push_back(A->rotationMat * aCollider->vecPoints[i] + A->position);


	}
	for (int i = 0; i < bSize; i++)
	{
		bPoints.push_back(B->rotationMat * bCollider->vecPoints[i] + B->position);


	}

	float bestDistanceA = -FLT_MAX;
	glm::vec2 bestContactPointA;
	glm::vec2 bestNormaleA;
	float bestDistanceB = -FLT_MAX;
	glm::vec2 bestContactPointB;
	glm::vec2 bestNormaleB;



	glm::vec2 X = glm::vec2(1, 0);
	glm::vec2 Y = glm::vec2(0, 1);
	glm::vec2 AX = A->rotationMat * X;
	glm::vec2 AY = A->rotationMat * Y;
	glm::vec2 BX = A->rotationMat * X;
	glm::vec2 BY = A->rotationMat * Y;
	float Ax_extent = aCollider->halfExtentX;
	float Ay_extent = aCollider->halfExtentY;
	float Bx_extent = bCollider->halfExtentX;
	float By_extent = bCollider->halfExtentY;
	float dotCX;
	float dotCY;
	bool BinA = false;
	bool AinB = false;
	glm::vec2 reach;
	bool inside = true;

	// Position of B relative to A
	glm::vec2 n = B->position - A->position;

	//We determine the closest point to B on A;
	glm::vec2 closest = n;
	dotCX = glm::dot(closest, AX); //We get closest coordinates in the basis of A
	dotCY = glm::dot(closest, AY);

	if (dotCX > Ax_extent)
	{
		dotCX = Ax_extent;
		inside = false;
	}
	else if (dotCX < -Ax_extent)
	{
		dotCX = -Ax_extent;
		inside = false;
	}

	if (dotCY > Ay_extent)
	{
		dotCY = Ay_extent;
		inside = false;
	}
	else if (dotCY < -Ay_extent)
	{
		dotCY = -Ay_extent;
		inside = false;
	}

	if (inside) //The center of B is inside A
	{
		if (abs(dotCX) > abs(dotCY))
		{
			if (dotCX > 0)
			{
				dotCX = Ax_extent;
			}
			else if (dotCX < 0)
			{
				dotCX = -Ax_extent;
			}
		}
		else
		{
			if (dotCY > 0)
			{
				dotCY = Ay_extent;
			}
			else if (dotCY < 0)
			{
				dotCY = -Ay_extent;
			}
		}

	}

	closest = dotCX * AX + dotCY * AY;

	glm::vec2 normal = n - closest;

	//Now we want to know the extent of B on normal
	float nExtent;
	//float nRadius;

	//Now we get the coordinates of closest in the B base
	
	

	if (dotCX > 0)
	{
		dotCX = Bx_extent;

	}
	else if (dotCX < 0)
	{
		dotCX = -Bx_extent;
	}


	if (dotCY > 0)
	{
		dotCY = By_extent;
	}
	else if (dotCY < 0)
	{
		dotCY = -By_extent;
	}

	nExtent = abs(glm::dot(glm::normalize(normal), dotCX * BX + dotCY * BY));

	float d = glm::length2(normal);

	if (nExtent * nExtent < d && !inside)
	{
		return false;
	}

	d = sqrt(d);

	if (inside)
	{
		m->penetration = nExtent + d;
		m->normal = -normal * (1 / d);
	}
	else
	{
		m->penetration = nExtent - d;
		m->normal = normal * (1 / d);
	}
	//nRadius




	return true;
	


}

bool resolveCollisionOBB_Circle(Manifold* m)
{
	// Setup a couple pointers to each object (le A doit être le rectangle et le B le Circle)
	RigidBody* A = m->A;
	RigidBody* B = m->B;

	OBBCollider* aCollider = dynamic_cast<OBBCollider*>(A->collider);
	CircleCollider* bCollider = dynamic_cast<CircleCollider*>(B->collider);
	glm::vec2 aX = A->rotationMat * glm::vec2(1, 0);
	glm::vec2 aY = A->rotationMat * glm::vec2(0, 1);
	// Vector from A to B
	glm::vec2 n = B->position - A->position;

	// Closest point on A to center of B
	glm::vec2 closest = n;
	bool inside = true;

	// Calculate half extents along each axis
	float x_extent = aCollider->halfExtentX;
	float y_extent = aCollider->halfExtentY;

	// Clamp point to edges of the OBB
	float dotCX = glm::dot(aX, closest);
	float dotCY = glm::dot(aY, closest);

	if (dotCX > x_extent)
	{
		dotCX = x_extent;
		inside = false;
	}
	else if (dotCX < -x_extent)
	{
		dotCX = -x_extent;
		inside = false;
	}
	if (dotCY > y_extent)
	{
		dotCY = y_extent;
		inside = false;
	}
	else if (dotCY < -y_extent)
	{
		dotCY = -y_extent;
		inside = false;
	}







	// Circle is inside the OBB, so we need to clamp the circle's center
	// to the closest edge
	if (inside)
	{



		// Find closest axis
		if (abs(dotCX) > abs(dotCY))
		{
			float previousDotCX = dotCX;
			// Clamp to closest extent
			if (dotCX > 0)
				dotCX = x_extent;
			else if(dotCX < 0)
				dotCX = -x_extent;
			

			////On calcule dotCY
			//if (dotCX == 0)
			//{
			//	//PAs sur que cette situation soit seulement possible
			//	dotCY = 1;
			//}
			//else
			//{
			//	
			//}

			// Based on the prior if statement, previousDotCX can't be equal to 0
			dotCY = (dotCY / previousDotCX) * dotCX;
		}

		// y axis is shorter
		else
		{
			float previousDotCY = dotCY;
			// Clamp to closest extent
			if (dotCY > 0)
				dotCY = y_extent;
			else
				dotCY = -y_extent;


			////On calcule dotCX
			//if (previousDotCY == 0)
			//{
			//	//PAs sur que cette situation soit seulement possible
			//	dotCX = 1;
			//}
			//else
			//{
			//	
			//}
			// Based on the prior if statement, previousDotCX can't be equal to 0
			dotCX = (dotCX / previousDotCY) * dotCY;
		}

	}

	closest = dotCX * aX + dotCY * aY;

	glm::vec2 normal = n - closest;
	float d = glm::length2(normal);
	float r = bCollider->radius;

	// Early out of the radius is shorter than distance to closest point and
	// Circle not inside the AABB
	if (d > r * r && !inside)
		return false;


	// Avoided sqrt until we needed
	d = sqrt(d);

	// Collision normal needs to be flipped to point outside if circle was
	// inside the AABB
	if (inside)
	{
		//m->normal = (Vector2d)(-normal) * (1 / d);
		m->normal = (-normal) * (1 / d);
		m->penetration = r - d;
	}
	else
	{
		m->normal = normal * (1 / d);
		m->penetration = r - d;
	}
	return true;
}


bool resolveCollisionCircle_OBB(Manifold* m)
{
	RigidBody* tmp = m->A;
	m->A = m->B;
	m->B = tmp;

	return resolveCollisionOBB_Circle(m);
}

bool resolveCollisionPolygonCircle(Manifold* m)
{
	RigidBody* A = m->A;
	RigidBody* B = m->B;


	PolygonCollider* aCollider = dynamic_cast<PolygonCollider*>(A->collider);
	PolygonCollider* bCollider = dynamic_cast<PolygonCollider*>(B->collider);
	return true;
}


bool resolveCollisionPolygonPolygon(Manifold* m)
{

	return true;
}

void actOnCollision(Manifold* m)
{
	// Pointers to our objects
	RigidBody* A = m->A;
	RigidBody* B = m->B;

	// Calculate relative velocity
	glm::vec2 rv = B->velocity - A->velocity;

	// Calculate relative velocity in terms of the normal direction
	float velAlongNormal = glm::dot(rv,m->normal); 	//rv.dotProduct(m->normal);

	

	// Do not resolve if velocities are separating
	if (velAlongNormal > 0)
	{
		Debug::log("fezfz");
		// Here 1 is an arbitrary value
	    if (glm::length2(rv) >= 1)
		{
			//Positional correction
			positionalCorrection(m);
			return;
		}
		

	}
	

	// Calculate restitution
	float e = std::min(A->restitution, B->restitution);

	// Calculate impulse scalar
	float j = -(1 + e) * velAlongNormal;
	j /= A->inv_mass + B->inv_mass;

	
	// Apply impulse
	glm::vec2 impulse = m->normal * j;

	//Apply impulse to object velocity
	A->velocity += (-impulse * A->inv_mass);
	B->velocity += (impulse * B->inv_mass);
	
	//We recalculate relative velocity
	rv = B->velocity - A->velocity;

	// Calculate normalized tangent vector 
	glm::vec2 t = rv -  m->normal * glm::dot(rv, m->normal);

	if (glm::length2(t) == 0)
	{
		positionalCorrection(m);
		return;
	}


	t = glm::normalize(t);
	
	
	
	
	float velAlongTangent = glm::dot(rv, t);
	float jt = -(1 + e) * velAlongTangent;
	jt /= A->inv_mass + B->inv_mass;

	//Now we apply Coulomp law, this part is optional and could be replaced with velocity decrease
	float mu = (A->staticFriction + B->staticFriction) / 2;// We do a simple average

	glm::vec2 frictionImpulse;
	if (std::abs(jt) < mu * j)
	{
		
		frictionImpulse = jt * t;
	}
	else
	{
		float dynamicFriction = (A->dynamicFriction + B->dynamicFriction) / 2;
		frictionImpulse = -j * t * dynamicFriction;
	}

	A->velocity += (-frictionImpulse * A->inv_mass);
	B->velocity += (frictionImpulse * B->inv_mass);
	

	//Positional correction
	positionalCorrection(m);

	
}





void positionalCorrection(Manifold* m)
{
	//If the collision is to important, we manually modify their position
	const float percent = 0.2f; // usually 20% to 80%
	const float slop = 0.05f; // usually 0.01 to 0.1
	glm::vec2 correction = m->normal * (percent * glm::max(m->penetration - slop, 0.0f) / (m->A->inv_mass + m->B->inv_mass));
	m->A->position += -correction * m->A->inv_mass;
	m->B->position += correction * m->B->inv_mass;
}


bool PolygonVSPolygonSAT(RigidBody* A, RigidBody* B)
{
	auto aCollider = dynamic_cast<PolygonCollider*>(A->collider);
	auto bCollider = dynamic_cast<PolygonCollider*>(B->collider);

	int aSize = aCollider->nbPoints;
	int bSize = bCollider->nbPoints;
	std::vector<glm::vec2> aPoints;
	std::vector<glm::vec2> bPoints;
	//On récupère les points dans l'espace
	for (int i = 0; i < aSize; i++)
	{
		aPoints.push_back(A->rotationMat * aCollider->vecPoints[i]+A->position);
		
	}
	for (int i = 0; i < bSize; i++)
	{
		bPoints.push_back(B->rotationMat * bCollider->vecPoints[i] + B->position);
		
	}
	//std::cout << "during that test aPoints contains" << aPoints.size() << "\n";
	//std::cout << "during that test aPoints contains" << bPoints.size() << "\n";
	//std::cout << "during that test asize is" << aSize << "\n";
	//std::cout << "during that test bsize is" << bSize << "\n";

	glm::vec2 currentAxis;
	//On passe par toutes les faces de A
	for (int i = 0; i < aSize; i++)
	{
		// la première vertex est celle d'indice i
		// la seconde est celle d'indice (i + 1) % aSize, on veut la seconde vertex de la face, le modulo permet de prendre en compte un indice = aSize ce qui n'est pas possible
		
		currentAxis.x = -(aPoints[(i + 1) % aSize].y - aPoints[i].y);
		currentAxis.y = (aPoints[(i + 1) % aSize].x - aPoints[i].x);
		
		
		//On obtient ainsi la normal à la face;
		float amin = FLT_MAX;
		float amax = -FLT_MAX;
		float bmin = FLT_MAX;
		float bmax= -FLT_MAX;
		float dot;

		for (int j = 0; j < aSize; j++)
		{

			dot = glm::dot(aPoints[j], currentAxis);
			
			if (dot > amax)
			{
				amax = dot;
			}
			if (dot < amin)
			{
				amin = dot;
			}
		}
		for (int j = 0; j < bSize; j++)
		{
			dot = glm::dot(bPoints[j], currentAxis);
			if (dot > bmax)
			{
				bmax = dot;
			}
			if (dot < bmin)
			{
				bmin = dot;
			}
		}
		if ((amin < bmax && amin > bmin) || (bmin < amax && bmin > amin))
		{
			continue;
		}
		else
		{
			//std::cout << amin << "\n";
			//std::cout << bmin << "\n";
			//std::cout << amax << "\n";
			//std::cout << bmax << "\n";
			return false;
		}

	}
	
	
	for (int i = 0; i < bSize; i++)
	{
		// la première vertex est celle d'indice i
		// la seconde est celle d'indice (i + 1) % aSize, on veut la seconde vertex de la face, le modulo permet de prendre en compte un indice = aSize ce qui n'est pas possible

		currentAxis.x = -(bPoints[(i + 1) % bSize].y - bPoints[i].y);
		currentAxis.y = (bPoints[(i + 1) % bSize].x - bPoints[i].x);
		//On obtient ainsi la normal à la face;
		float amin = FLT_MAX;
		float amax = -FLT_MAX;
		float bmin = FLT_MAX;
		float bmax = -FLT_MAX;
		float dot;

		for (int j = 0; j < aSize; j++)
		{
			dot = glm::dot(aPoints[j], currentAxis);
			if (dot > amax)
			{
				amax = dot;
			}
			if (dot < amin)
			{
				amin = dot;
			}
		}
		for (int j = 0; j < bSize; j++)
		{
			dot = glm::dot(bPoints[j], currentAxis);
			if (dot > bmax)
			{
				bmax = dot;
			}
			if (dot < bmin)
			{
				bmin = dot;
			}
		}
		if ((amin < bmax && amin > bmin) || (bmin < amax && bmin > amin))
		{
			continue;
		}
		else
		{
			return false;
		}

	}




	return true;
}


bool OBBCollisionTest(RigidBody* A, RigidBody* B)
{
	auto aCollider = dynamic_cast<OBBCollider*>(A->collider);
	auto bCollider = dynamic_cast<OBBCollider*>(B->collider);

	int aSize = aCollider->nbPoints;
	int bSize = bCollider->nbPoints;
	std::vector<glm::vec2> aPoints;
	std::vector<glm::vec2> bPoints;

	for (int i = 0; i < aSize; i++)
	{
		aPoints.push_back(A->rotationMat * aCollider->vecPoints[i] + A->position);
		

	}
	for (int i = 0; i < bSize; i++)
	{
		bPoints.push_back(B->rotationMat * bCollider->vecPoints[i] + B->position);
	

	}

	float bestDistance = -FLT_MAX;
	int bestIndice = -1;


	glm::vec2 X = glm::vec2(1,0);
	glm::vec2 Y = glm::vec2(0,1);
	glm::vec2 xAxis;
	glm::vec2 yAxis;
	glm::vec2 currentAxis;
	float currentDistance;
	int currentIndice;



	////Pour l'instant on fait ça de manière bourrine mais il faudra mettre ça dans une fonction
	////Test par rapport aux axes de A

	xAxis = A->rotationMat * X;
	yAxis = A->rotationMat * Y;
	
	currentAxis = -xAxis;
	currentIndice = PhysicData::getSupportPoint(bPoints, currentAxis);
	//Debug::log("currentI", currentIndice);
	currentDistance = glm::dot(bPoints[currentIndice] - aPoints[3], -currentAxis);

	if (currentDistance > bestDistance)
	{
		bestDistance = currentDistance;
		bestIndice = currentIndice;
	}

	currentAxis = -yAxis;
	currentIndice = PhysicData::getSupportPoint(bPoints, currentAxis);
	currentDistance = glm::dot(bPoints[currentIndice] - aPoints[2], -currentAxis);

	if (currentDistance > bestDistance)
	{
		bestDistance = currentDistance;
		bestIndice = currentIndice;
	}

	currentAxis = xAxis;
	currentIndice = PhysicData::getSupportPoint(bPoints, currentAxis);
	currentDistance = glm::dot(bPoints[currentIndice] - aPoints[1], -currentAxis);

	if (currentDistance > bestDistance)
	{
		bestDistance = currentDistance;
		bestIndice = currentIndice;
	}

	currentAxis = yAxis;
	currentIndice = PhysicData::getSupportPoint(bPoints, currentAxis);
	currentDistance = glm::dot(bPoints[currentIndice] - aPoints[0],-currentAxis);

	if (currentDistance > bestDistance)
	{
		bestDistance = currentDistance;
		bestIndice = currentIndice;
	}






	//Test par rapport aux axes de B
	xAxis = B->rotationMat * X;
	yAxis = B->rotationMat * Y;

	currentAxis = -xAxis;
	currentIndice = PhysicData::getSupportPoint(aPoints, currentAxis);
	currentDistance = glm::dot(aPoints[currentIndice] - bPoints[3], -currentAxis);

	if (currentDistance > bestDistance)
	{
		bestDistance = currentDistance;
		bestIndice = currentIndice;
	}

	currentAxis = -yAxis;
	currentIndice = PhysicData::getSupportPoint(aPoints, currentAxis);
	currentDistance = glm::dot(aPoints[currentIndice] - bPoints[2], -currentAxis);

	if (currentDistance > bestDistance)
	{
		bestDistance = currentDistance;
		bestIndice = currentIndice;
	}

	currentAxis = xAxis;
	currentIndice = PhysicData::getSupportPoint(aPoints, currentAxis);
	currentDistance = glm::dot(aPoints[currentIndice] - bPoints[1], -currentAxis);

	if (currentDistance > bestDistance)
	{
		bestDistance = currentDistance;
		bestIndice = currentIndice;
	}

	currentAxis = yAxis;
	currentIndice = PhysicData::getSupportPoint(aPoints, currentAxis);
	currentDistance = glm::dot(aPoints[currentIndice] - bPoints[0], -currentAxis);

	if (currentDistance > bestDistance)
	{
		bestDistance = currentDistance;
		bestIndice = currentIndice;
	}
	//Debug::log("penetration", bestDistance);
//PhysicData::getSupportPoint(aPoints, glm::vec2(1, 0));

	if (bestDistance >= 0)
	{
		
		return false;
	}
	else
	{
		
		return true;
	}

}

//void findAxisOfLeastPenetrationOBB(glm::vec2& xAxis, glm::vec2& yAxis,std::vector<glm::vec2>& points, float* bestDistance, int* bestIndice)
//{
//	glm::vec2 currentAxis;
//	int currentIndice;
//	float currentDistance;
//	for (int i = 0; i < 2; i++)
//	{
//		currentAxis = (float)(i * 2 -1) * xAxis;
//		currentIndice = PhysicData::getSupportPoint(points, currentAxis);
//		currentDistance = glm::dot(aPoints[currentIndice] - bPoints[0], -currentAxis);
//	}
//	/*int currentIndice = PhysicData::getSupportPoint(points, currentAxis);
//	float currentDistance = glm::dot(aPoints[currentIndice] - bPoints[3], -currentAxis);
//
//	if (currentDistance > bestDistance)
//	{
//		bestDistance = currentDistance;
//		bestIndice = currentIndice;
//	}*/
//}







//Previous algorithm for OBB collision, did not work with angles

//	//// Setup a couple pointers to each object
//RigidBody* A = m->A;
//RigidBody* B = m->B;
//
//OBBCollider* aCollider = dynamic_cast<OBBCollider*>(A->collider);
//OBBCollider* bCollider = dynamic_cast<OBBCollider*>(B->collider);
//
////// Vector from A to B
//glm::vec2 n = A->position - B->position;
//
//
//
////// Calculate half extents along x axis for each object
//float a_extent = aCollider->halfExtentX;
//float b_extent = bCollider->halfExtentX;
//
////// Calculate overlap on x axis
//float x_overlap = a_extent + b_extent - abs(n.x);
//
//// SAT test on x axis
//if (x_overlap > 0)
//{
//	// Calculate half extents along y axis for each object
//	float a_extent = aCollider->halfExtentY;
//	float b_extent = bCollider->halfExtentY;
//
//	// Calculate overlap on y axis
//	float y_overlap = a_extent + b_extent - abs(n.y);
//
//
//	// SAT test on y axis
//	if (y_overlap > 0)
//	{
//		// Find out which axis is axis of least penetration
//		if (x_overlap < y_overlap)
//		{
//			// Point towards B knowing that n points from A to B
//			if (n.x < 0)
//			{
//				m->normal = glm::vec2(1, 0);
//			}
//
//			else
//			{
//				m->normal = glm::vec2(-1, 0);
//			}
//
//			m->penetration = x_overlap;
//
//			return true;
//		}
//		else
//		{
//			// Point toward B knowing that n points from A to B
//			if (n.y < 0)
//			{
//				m->normal = glm::vec2(0, 1);
//			}
//
//			else
//			{
//				m->normal = glm::vec2(0, -1);
//			}
//
//			m->penetration = y_overlap;
//			return true;
//		}
//	}
//}
//
//return false;



bool ff(Manifold* m)
{
	

	RigidBody* A = m->A;
	RigidBody* B = m->B;
	auto aCollider = dynamic_cast<OBBCollider*>(A->collider);
	auto bCollider = dynamic_cast<OBBCollider*>(B->collider);

	int aSize = aCollider->nbPoints;
	int bSize = bCollider->nbPoints;
	std::vector<glm::vec2> aPoints;
	std::vector<glm::vec2> bPoints;

	for (int i = 0; i < aSize; i++)
	{
		aPoints.push_back(A->rotationMat * aCollider->vecPoints[i] + A->position);


	}
	for (int i = 0; i < bSize; i++)
	{
		bPoints.push_back(B->rotationMat * bCollider->vecPoints[i] + B->position);


	}

	float bestDistanceA = -FLT_MAX;
	glm::vec2 bestContactPointA;
	glm::vec2 bestNormaleA;
	float bestDistanceB = -FLT_MAX;
	glm::vec2 bestContactPointB;
	glm::vec2 bestNormaleB;



	glm::vec2 X = glm::vec2(1, 0);
	glm::vec2 Y = glm::vec2(0, 1);
	glm::vec2 AX = A->rotationMat * X;
	glm::vec2 AY = A->rotationMat * Y;
	glm::vec2 BX = A->rotationMat * X;
	glm::vec2 BY = A->rotationMat * Y;
	float Ax_extent = aCollider->halfExtentX;
	float Ay_extent = aCollider->halfExtentY;
	float Bx_extent = bCollider->halfExtentX;
	float By_extent = bCollider->halfExtentY;
	float dotCX;
	float dotCY;
	bool BinA = false;
	bool AinB = false;
	glm::vec2 reach;
	bool inside = true;
	
	// Position of B relative to A
	glm::vec2 n = B->position - A->position;

	//We determine the closest point to B on A;
	glm::vec2 closest = n;
	dotCX = glm::dot(closest, AX); //We get closest coordinates in the basis of A
	dotCY = glm::dot(closest, AY);

	if (dotCX > Ax_extent)
	{
		dotCX = Ax_extent;
		inside = false;
	}
	else if (dotCX < -Ax_extent)
	{
		dotCX = -Ax_extent;
		inside = false;
	}

	if (dotCY > Ay_extent)
	{
		dotCY = Ay_extent;
		inside = false;
	}
	else if (dotCY < -Ay_extent)
	{
		dotCY = -Ay_extent;
		inside = false;
	}

	if (inside) //The center of B is inside A
	{
		if (abs(dotCX) > abs(dotCY))
		{
			if (dotCX > 0)
			{
				dotCX = Ax_extent;
			}
			else if (dotCX < 0)
			{
				dotCX = -Ax_extent;
			}
		}
		else
		{
			if (dotCY > 0)
			{
				dotCY = Ay_extent;
			}
			else if (dotCY < 0)
			{
				dotCY = -Ay_extent;
			}
		}
		
	}

	closest = dotCX * AX + dotCY * AY;

	glm::vec2 normal = n - closest;

	//Now we want to know the extent of B on normal
	float nExtent;
	//float nRadius;
	
	//Now we get the coordinates of closest in the B base
	dotCX = glm::dot(BX, normal);
	dotCY = glm::dot(BY, normal);

	if (dotCX > 0)
	{
		dotCX = Bx_extent;
		
	}
	else if (dotCX < 0)
	{
		dotCX = -Bx_extent;
	}
	

	if (dotCY > 0)
	{
		dotCY = By_extent;
	}
	else if (dotCY < 0)
	{
		dotCY = -By_extent;
	}
	
	nExtent = glm::dot(normal, dotCX * BX + dotCY * BY);

	float d = glm::length2(normal);

	if (nExtent * nExtent < d && !inside)
	{
		return false;
	}

	d = sqrt(d);

	if (inside)
	{
		m->penetration = nExtent - d;
		m->normal = -normal * (1 / d);
	}
	else
	{
		m->penetration = nExtent - d;
		m->normal = normal * (1 / d);
	}
	//nRadius
	
	

	
	return true;


}