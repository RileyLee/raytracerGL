#include "ray.h"
#include "material.h"
#include "light.h"

#include "../fileio/imageio.h"

using namespace std;
extern bool debugMode;


// Apply the Blinn-Phong model to this point on the surface of the object, 
//  returning the color of that point.
Vec3d Material::shade( Scene *scene, const ray& r, const isect& i) const
{

	if( debugMode )
		std::cout << "Debugging the Phong code (or lack thereof...)" << std::endl;

	Vec3d toRet = ke(i) + (prod(ka(i), scene->ambient()));
	Vec3d Q = r.at(i.t);
	Vec3d N = i.N;
	if (r.type() == 2)
		N = -N;
	N.normalize();
	double _shininess = this->shininess(i);

	for ( vector<Light*>::const_iterator litr = scene->beginLights(); 
		  litr != scene->endLights(); 
		  ++litr ) {
	  Light* pLight = *litr;
	  Vec3d lightDirection = pLight->getDirection(Q);
	  lightDirection.normalize();
	  if(N * lightDirection <= 0.0) {
		continue;
	  }
	  Vec3d V = -r.getDirection();
	  /*
	  ray lightingRay = ray(Q, lightDirection,  (enum ray::RayType)3);
	  isect i;
	  scene->intersect(lightingRay, i);
	  */
	  V.normalize();
	  Vec3d H = (V + lightDirection) / 2;
	  H.normalize();
	  double nDotH = max(0.0, N * H);
	  Vec3d shadowAtten = pLight->shadowAttenuation(Q);
	  Vec3d reflectionCoeff = pLight->distanceAttenuation(Q) * ((kd(i) * (N * lightDirection)) + (ks(i) * (pow(nDotH, _shininess))));
	  reflectionCoeff = prod(shadowAtten, reflectionCoeff);
	  toRet += prod(pLight->getColor(), reflectionCoeff);
	}

	return toRet;
}


TextureMap::TextureMap( string filename )
{
    data = load( filename.c_str(), width, height );
    if( 0 == data )
    {
        width = 0;
        height = 0;
        string error( "Unable to load texture map '" );
        error.append( filename );
        error.append( "'." );
        throw TextureMapException( error );
    }
}

Vec3d TextureMap::getMappedValue( const Vec2d& coord ) const
{
	// YOUR CODE HERE

    // In order to add texture mapping support to the 
    // raytracer, you need to implement this function.
    // What this function should do is convert from
    // parametric space which is the unit square
    // [0, 1] x [0, 1] in 2-space to Image coordinates,
    // and use these to perform bilinear interpolation
    // of the values.


    return Vec3d(1.0, 1.0, 1.0);
}


Vec3d TextureMap::getPixelAt( int x, int y ) const
{
    // This keeps it from crashing if it can't load
    // the texture, but the person tries to render anyway.
    if (0 == data)
      return Vec3d(1.0, 1.0, 1.0);

    if( x >= width )
       x = width - 1;
    if( y >= height )
       y = height - 1;

    // Find the position in the big data array...
    int pos = (y * width + x) * 3;
    return Vec3d( double(data[pos]) / 255.0, 
       double(data[pos+1]) / 255.0,
       double(data[pos+2]) / 255.0 );
}

Vec3d MaterialParameter::value( const isect& is ) const
{
    if( 0 != _textureMap )
        return _textureMap->getMappedValue( is.uvCoordinates );
    else
        return _value;
}

double MaterialParameter::intensityValue( const isect& is ) const
{
    if( 0 != _textureMap )
    {
        Vec3d value( _textureMap->getMappedValue( is.uvCoordinates ) );
        return (0.299 * value[0]) + (0.587 * value[1]) + (0.114 * value[2]);
    }
    else
        return (0.299 * _value[0]) + (0.587 * _value[1]) + (0.114 * _value[2]);
}