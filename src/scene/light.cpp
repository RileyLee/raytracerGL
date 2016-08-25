#include <cmath>

#include "light.h"


using namespace std;

//#define SINGLE_SIDED

extern bool debugMode;

double DirectionalLight::distanceAttenuation( const Vec3d& P ) const
{
	// distance to light is infinite, so f(di) goes to 0.  Return 1.
	return 1.0;
}


Vec3d DirectionalLight::shadowAttenuation( const Vec3d& P ) const
{
  Scene *scene = getScene();
  Vec3d toReturn(1,1,1);
  Vec3d currP = P;
  Vec3d lightDirection = getDirection(P);
  while(true) {
	isect i;
	ray r(currP, lightDirection, (enum ray::RayType)3);
	bool intersection = scene->intersect(r, i);
	if(!intersection) {
	  return toReturn;
	}
	const Material &m = i.getMaterial();
	toReturn = prod(toReturn, m.kt(i));
	if(toReturn.length() == 0) {
	  return toReturn;
	}
	currP = r.at(i.t);
  }
}

Vec3d DirectionalLight::getColor() const
{
	return color;
}

Vec3d DirectionalLight::getDirection( const Vec3d& P ) const
{
	return -orientation;
}

double PointLight::distanceAttenuation( const Vec3d& P ) const
{
  Vec3d l = position - P;
  double denom = constantTerm + linearTerm * l.length() + quadraticTerm * l.length2();
  return min(1.0, 1.0 / denom);
}

Vec3d PointLight::getColor() const
{
	return color;
}

Vec3d PointLight::getDirection( const Vec3d& P ) const
{
	Vec3d ret = position - P;
	ret.normalize();
	return ret;
}


Vec3d PointLight::shadowAttenuation(const Vec3d& P) const
{
  Scene *scene = getScene();
  Vec3d toReturn(1,1,1);
  Vec3d currP = P;
  Vec3d lightDirection = getDirection(P);
  double maxT;
  {
	Vec3d lightVec = position - P;
	maxT = lightVec.length();
	ray r(P, lightDirection);
	Vec3d test = r.at(maxT) - position;
	assert(test.length() < RAY_EPSILON);
  }
  while(true) {
	isect i;
	ray r(currP, lightDirection, (enum ray::RayType)3);
	bool intersection = scene->intersect(r, i);
	if(!intersection) {
	  return toReturn;
	}
	if(i.t > maxT) {
	  return toReturn;
	}
	const Material &m = i.getMaterial();
	toReturn = prod(toReturn, m.kt(i));
	if(toReturn.length() == 0) {
	  return toReturn;
	}
	currP = r.at(i.t);
  }
}
