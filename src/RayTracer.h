#ifndef __RAYTRACER_H__
#define __RAYTRACER_H__

// The main ray tracer.

#include "scene/ray.h"

#define THREAD_CHUNKSIZE 32

class Scene;

class RayTracer
{
public:
    RayTracer();
    ~RayTracer();

    Vec3d trace( double x, double y );
	Vec3d traceRay( const ray& r, const Vec3d& thresh, int depth );


	void getBuffer( unsigned char *&buf, int &w, int &h );
	double aspectRatio();
	void traceSetup( int w, int h );
	void tracePixel( int i, int j );
	void tracePixelAntiAlias(int i, int j);
	bool Antialias;
	void(RayTracer::*callTracePixel_ptr)(int, int) const = NULL;
	//callTracePixel_ptr cb_tracer;
	//void(*ptrTracePixel)(int, int);
	bool notTIR(const double n_i, const double n_t, Vec3d d, Vec3d N);
	Vec3d reflectDirection(const Vec3d& N, const Vec3d& d) const;
	Vec3d refractDirection(const double n_i, const double n_t, const Vec3d& N, const Vec3d& d) const;

	bool loadScene( const char* fn );

	bool sceneLoaded() { return scene != 0; }

	//void set_cb_tracer(callTracePixel_ptr ptr);
	//void call_cb_tracer(int x, int y);

    void setReady( bool ready )
      { m_bBufferReady = ready; }
    bool isReady() const
      { return m_bBufferReady; }

	const Scene& getScene() { return *scene; }

private:
	unsigned char *buffer;
	int buffer_width, buffer_height;
	int bufferSize;
	Scene* scene;

    bool m_bBufferReady;

};

#endif // __RAYTRACER_H__
