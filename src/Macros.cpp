#include "Macros.h"

float crossVec2(const glm::vec2& a, const glm::vec2& b)
{
	return (a.x * b.y - a.y * b.x);
}

glm::vec2 crossVec2(float a, const glm::vec2& b)
{
	return glm::vec2(a*b.y, -a*b.x);
}