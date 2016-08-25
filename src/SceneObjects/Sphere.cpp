#include <cmath>

#include "Sphere.h"

using namespace std;


bool Sphere::intersectLocal( const ray& r, isect& i ) const
{
	Vec3d p = r.getPosition();
	Vec3d d = r.getDirection();

	double a = d.length2();
	double b = p * d * 2;
	double c = p.length2() - 1;

	double delta = b * b - 4 * a * c;

	if (delta < 0)
		return false;
	delta = sqrt(delta);
	
	double t;
	t = min((-b - delta) / 2 / a, (-b + delta) / 2 / a);
	if(t < RAY_EPSILON) {
	  return false;
	}


	Vec3d pt = r.at(t) * 2;
	pt.normalize();

	i.setN(pt);
	i.setT(t);
	i.setObject(this);

	return true;

}

