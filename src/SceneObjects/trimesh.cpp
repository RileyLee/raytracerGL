#include <cmath>
#include <float.h>
#include "trimesh.h"

using namespace std;

Trimesh::~Trimesh()
{
	for( Materials::iterator i = materials.begin(); i != materials.end(); ++i )
		delete *i;
}

bool Trimesh::hasPerVertexNormals()
{
	return !(this->normals.empty());
}

// must add vertices, normals, and materials IN ORDER
void Trimesh::addVertex( const Vec3d &v )
{
    vertices.push_back( v );
}

void Trimesh::addMaterial( Material *m )
{
    materials.push_back( m );
}

void Trimesh::addNormal( const Vec3d &n )
{
    normals.push_back( n );
}

void Trimesh::addTextureUV( const Vec2d &n )
{
    textureuvs.push_back( n );
}

// Returns false if the vertices a,b,c don't all exist
bool Trimesh::addFace( int a, int b, int c )
{
    int vcnt = vertices.size();

    if( a >= vcnt || b >= vcnt || c >= vcnt )
        return false;

    TrimeshFace *newFace = new TrimeshFace( scene, new Material(*this->material), this, a, b, c );
    newFace->setTransform(this->transform);
    faces.push_back( newFace );
    scene->add(newFace);
    return true;
}

char *
Trimesh::doubleCheck()
// Check to make sure that if we have per-vertex materials or normals
// they are the right number.
{
    if( !materials.empty() && materials.size() != vertices.size() )
        return "Bad Trimesh: Wrong number of materials.";
    if( !normals.empty() && normals.size() != vertices.size() )
        return "Bad Trimesh: Wrong number of normals.";

    return 0;
}

extern bool debugMode;

// Calculates and returns the normal of the triangle too.
bool TrimeshFace::intersectLocal( const ray& r, isect& i ) const
{
  Vec3d &a = parent->vertices[ids[0]];
  Vec3d &b = parent->vertices[ids[1]];
  Vec3d &c = parent->vertices[ids[2]];

  const Vec3d bSubA = b - a;
  const Vec3d cSubA = c - a;

  Vec3d abc_cross = bSubA ^ cSubA;
  
  Vec3d n = abc_cross;
  if(n.length() == 0) {
	return false;
  }
  n.normalize();
  double plane_d = n * a;
  
  double denom = n * r.getDirection();
  if(denom == 0) {
	return false;
  }

  double num = plane_d - n * r.getPosition();
  double t = num / denom;
  if(t < RAY_EPSILON) {
	return false;
  }

  Vec3d q = r.at(t);
  double abCrossAQ = (bSubA ^ (q - a)) * n;
  double bcCrossBQ = ((c - b) ^ (q - b)) * n;
  double caCrossCQ = ((a - c) ^ (q - c)) * n;
  if(abCrossAQ < 0 || bcCrossBQ < 0 || caCrossCQ < 0) {
	return false;
  }

  // if(debugMode) {
  // 	std::cout << ids[0] << " " << ids[1] << " " << ids[2] << std::endl;
  // 	std::cout << a << ", " << b << ", " << c << std::endl;
  // }
  
  double baryDenom =  abc_cross * n;
  double alpha = bcCrossBQ / baryDenom;
  double beta = caCrossCQ / baryDenom;
  double gamma = abCrossAQ / baryDenom;
  i.obj = this;
  i.t = t;
  i.setMaterial(*this->material);
  if(parent->hasPerVertexNormals()) {
	Vec3d n_q = alpha * parent->normals.at(ids[0]) + 
	  beta * parent->normals.at(ids[1]) + 
	  gamma * parent->normals.at(ids[2]);
	n_q.normalize();
	i.setN(n_q);
  } else {
	i.setN(n);
  }
  return true;
}


void
Trimesh::generateNormals()
// Once you've loaded all the verts and faces, we can generate per
// vertex normals by averaging the normals of the neighboring faces.
{
    int cnt = vertices.size();
    normals.resize( cnt );
    int *numFaces = new int[ cnt ]; // the number of faces assoc. with each vertex
    memset( numFaces, 0, sizeof(int)*cnt );
    
    for( Faces::iterator fi = faces.begin(); fi != faces.end(); ++fi )
    {
        Vec3d a = vertices[(**fi)[0]];
        Vec3d b = vertices[(**fi)[1]];
        Vec3d c = vertices[(**fi)[2]];
        
        Vec3d faceNormal = ((b-a) ^ (c-a));
		faceNormal.normalize();
        
        for( int i = 0; i < 3; ++i )
        {
            normals[(**fi)[i]] += faceNormal;
            ++numFaces[(**fi)[i]];
        }
    }

    for( int i = 0; i < cnt; ++i )
    {
        if( numFaces[i] )
		{
            normals[i]  /= numFaces[i];
			if (normals[i].length() != 0)
			{
				normals[i].normalize();
			}
		}
    }

    delete [] numFaces;
}

