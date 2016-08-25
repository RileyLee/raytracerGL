// The main ray tracer.

#pragma warning (disable: 4786)

#include "RayTracer.h"
#include "scene/light.h"
#include "scene/material.h"
#include "scene/ray.h"

#include "parser/Tokenizer.h"
#include "parser/Parser.h"

#include "ui/TraceUI.h"
#include <cmath>
#include <algorithm>

extern TraceUI* traceUI;

#include <iostream>
#include <fstream>

using namespace std;

// Use this variable to decide if you want to print out
// debugging messages.  Gets set in the "trace single ray" mode
// in TraceGLWindow, for example.
bool debugMode = false;

// Trace a top-level ray through normalized window coordinates (x,y)
// through the projection plane, and out into the scene.  All we do is
// enter the main ray-tracing method, getting things started by plugging
// in an initial ray weight of (0.0,0.0,0.0) and an initial recursion depth of 0.
Vec3d RayTracer::trace( double x, double y )
{
	// Clear out the ray cache in the scene for debugging purposes,
	if (!traceUI->isMultithreading())
		scene->intersectCache.clear();

    ray r( Vec3d(0,0,0), Vec3d(0,0,0), ray::VISIBILITY );

    scene->getCamera().rayThrough( x,y,r );
	Vec3d ret = traceRay( r, Vec3d(1.0,1.0,1.0), 0 );
	ret.clamp();
	return ret;
}

// Do recursive ray tracing!  You'll want to insert a lot of code here
// (or places called from here) to handle reflection, refraction, etc etc.
Vec3d RayTracer::traceRay( const ray& r, 
	const Vec3d& thresh, int depth )
{
	isect i;
	double n_i, n_t;
	Vec3d Q, I, tempD;
	int depthLeft = traceUI->getDepth() - depth;

	if (scene->intersect(r, i)) {

		Q = r.at(i.t);

		const Material& m = i.getMaterial();
		I = m.shade(scene, r, i);
		depthLeft = traceUI->getDepth() - depth;
		if (depthLeft > 0){
			if (m.kr(i).length() > 0){
				Vec3d R = reflectDirection(i.N, -r.getDirection());

				ray r_reflection(Q, R, ray::REFLECTION);

				I = I + prod(m.kr(i), traceRay(r_reflection, thresh, depth + 1));

			}
			
			if ((r.getDirection()* i.N) < 0){
				n_i = 1.003;  n_t = m.index(i); depth++; tempD = i.N;
			}
			else{
				n_i = m.index(i); n_t = 1.003;  tempD = -i.N;
			}

			if ((notTIR(n_i, n_t, -r.getDirection(), i.N) & (m.kt(i).length()>0))){

				Vec3d T = refractDirection(n_i, n_t, tempD, r.getDirection());
				ray::RayType type = ray::REFRACTION;
				if ((r.getDirection()* i.N) > 0) type = ray::VISIBILITY;
				ray r_refraction(Q, T, type);     // The incoming ray from the first lens layer is not intersecting with the second wall of the same lens. It bounced back from the other objects.
				I = I + prod(m.kt(i), traceRay(r_refraction, thresh, depth));
			}
		}
		return I;
	}
	else {
		return Vec3d(0.0, 0.0, 0.0);
	}
}

RayTracer::RayTracer()
	: scene( 0 ), buffer( 0 ), buffer_width( 0 ), buffer_height( 0 ), m_bBufferReady( false )
{
}


RayTracer::~RayTracer()
{
	delete scene;
	delete [] buffer;
}

void RayTracer::getBuffer( unsigned char *&buf, int &w, int &h )
{
	buf = buffer;
	w = buffer_width;
	h = buffer_height;
}

double RayTracer::aspectRatio()
{
	return sceneLoaded() ? scene->getCamera().getAspectRatio() : 1;
}

bool RayTracer::loadScene( const char* fn )
{
	ifstream ifs( fn );
	if( !ifs ) {
		string msg( "Error: couldn't read scene file " );
		msg.append( fn );
		traceUI->alert( msg );
		return false;
	}
	
	// Strip off filename, leaving only the path:
	string path( fn );
	if( path.find_last_of( "\\/" ) == string::npos )
		path = ".";
	else
		path = path.substr(0, path.find_last_of( "\\/" ));

	// Call this with 'true' for debug output from the tokenizer
	Tokenizer tokenizer( ifs, false );
    Parser parser( tokenizer, path );
	try {
		delete scene;
		scene = 0;
		scene = parser.parseScene();
	} 
	catch( SyntaxErrorException& pe ) {
		traceUI->alert( pe.formattedMessage() );
		return false;
	}
	catch( ParserException& pe ) {
		string msg( "Parser: fatal exception " );
		msg.append( pe.message() );
		traceUI->alert( msg );
		return false;
	}
	catch( TextureMapException e ) {
		string msg( "Texture mapping exception: " );
		msg.append( e.message() );
		traceUI->alert( msg );
		return false;
	}


	if( ! sceneLoaded() )
		return false;

	scene->indexObjects();

	
	return true;
}

void RayTracer::traceSetup( int w, int h )
{
	if( buffer_width != w || buffer_height != h )
	{
		buffer_width = w;
		buffer_height = h;

		bufferSize = buffer_width * buffer_height * 3;
		delete [] buffer;
		buffer = new unsigned char[ bufferSize ];

	}
	memset( buffer, 0, w*h*3 );
	m_bBufferReady = true;

	//callTracePixel_ptr = &tracePixel;
	//callTracePixel_ptr cb_tracer = &RayTracer::tracePixel;
	//set_cb_tracer(cb_tracer);
}

void RayTracer::tracePixel( int i, int j )
{
	Vec3d col;

	if( ! sceneLoaded() )
		return;

	double x = double(i)/double(buffer_width);
	double y = double(j)/double(buffer_height);

	col = trace( x, y);

	unsigned char *pixel = buffer + ( i + j * buffer_width ) * 3;

	pixel[0] = (int)( 255.0 * col[0]);
	pixel[1] = (int)( 255.0 * col[1]);
	pixel[2] = (int)( 255.0 * col[2]);
}

void RayTracer::tracePixelAntiAlias(int i, int j)
{
	Vec3d col(0,0,0);

	double subsamplrate = 3;

	if (!sceneLoaded())
		return;

	double space = 1 / subsamplrate;
	double xa, ya;
	for (int numX = 0; numX < subsamplrate; numX++){
		for (int numY = 0; numY < subsamplrate; numY++){
			xa = ((double(i) - 0.5) + space / 2 + double(numX)*space) / double(buffer_width);
			ya = ((double(j) - 0.5) + space / 2 + double(numY)*space) / double(buffer_height);
			col += trace(xa, ya);
		}
	}


	

	unsigned char *pixel = buffer + (i + j * buffer_width) * 3;

	pixel[0] = (int)(255.0 * col[0] / subsamplrate / subsamplrate);
	pixel[1] = (int)(255.0 * col[1] / subsamplrate / subsamplrate);
	pixel[2] = (int)(255.0 * col[2] / subsamplrate / subsamplrate);
}

/*void RayTracer::set_cb_tracer(callTracePixel_ptr ptr){
	cb_tracer = ptr;
}

void RayTracer::call_cb_tracer(int x, int y){
	(this->cb_tracer)(x, y);
}*/

bool RayTracer::notTIR(const double n_i, const double n_t, Vec3d d, Vec3d N)
{
	Vec3d cross = d^N;
	double sini = cross.length() / d.length() / N.length();
	return (sini < (n_t / n_i));
}

Vec3d RayTracer::reflectDirection(const Vec3d& N, const Vec3d& d) const
{
	Vec3d R = (2 * N - d);
	R.normalize();
	return R;
}

Vec3d RayTracer::refractDirection(const double n_i, const double n_t, const Vec3d& N, const Vec3d& d) const
{
	double cosI = N*d;
	double ratio = n_i / n_t;
	double sinT2 = ratio*ratio*(1 - cosI*cosI);
	Vec3d T = ratio * d - (ratio*cosI + sqrt(1 - sinT2))*N;
	T.normalize();
	return T;
}

