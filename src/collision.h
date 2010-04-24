#ifndef COLLISION_H
#define COLLISION_H

#include <vector>
#include <Newton.h>
#include "world.h"


struct RayCollision
{
	dFloat distance;
	dFloat normalX, normalY, normalZ;
	int attributeID;

};

class CollisionGeometry
{
protected:
	NewtonCollision* m_collision;
	NewtonWorld* m_world;

public:
	CollisionGeometry();
	virtual ~CollisionGeometry();

	NewtonCollision* GetNewtonCollision();
	NewtonWorld* GetNewtonWorld();

	//returns contact and normal as a tuple if possible, otherwise returns None
	// (contact_x, contact_y, contact_z, normal_x, normal_y, normal_z )
	PyObject* PointDistance ( const dFloat *point, const dFloat* matrix);

	//returns a tuple of 9 values if possible, otherwise returns None
	PyObject* ClosestPoint ( const dFloat* matrixA, CollisionGeometry* collsionB, const dFloat* matrixB );

	//Returns a tuple of 6 numbers representing the bounds of the AABB:
	// (min_x, min_y, min_z, max_x, max_y, max_z )
	PyObject* CalculateAABB( const dFloat* matrix );

	//returns list containing: [ distance, (nx, ny, nz), attributeID ] 
	RayCollision RayCast ( const dFloat* vectorA, const dFloat* vectorB ) ;
};

class CompoundCollision : public CollisionGeometry
{
	std::vector < NewtonCollision* > m_geoms; 
public:
	CompoundCollision( World* world );

	void AddCollision( CollisionGeometry* c );
	void Finalize();
};

//Colliders 
class Sphere : public CollisionGeometry
{
public:
	Sphere( World* world, dFloat radiusX, dFloat radiusY, dFloat radiusZ, const dFloat* offsetMatrix );
};

class Box : public CollisionGeometry
{
public:
	Box(World* world, dFloat x, dFloat y, dFloat z, const dFloat* offsetMatrix );
};

class Cone: public CollisionGeometry
{
public:
	Cone( World* world, dFloat radius, dFloat height, const dFloat* offsetMatrix );
};

class Capsule : public CollisionGeometry
{
public: 
	Capsule( World* world, dFloat radius, dFloat height, const dFloat* offsetMatrix );
};

class Cylinder : public CollisionGeometry
{
public:
	Cylinder( World* world, dFloat radius, dFloat height, const dFloat* offsetMatrix );
};

class ChamferCylinder : public CollisionGeometry
{
public:
	ChamferCylinder( World* world, dFloat radius, dFloat height, const dFloat* offsetMatrix );
};

class ConvexHull : public CollisionGeometry
{

public:
	ConvexHull ( World* world, PyObject* pylist,  const dFloat* offsetMatrix  );
   
	void SetUserID( unsigned id );
	unsigned int GetUserID();

	dFloat CalculateVolume( );
	PyObject* CalculateInertiaMatrix ( );
   
};

class ConvexHullModifier : public CollisionGeometry
{
	ConvexHull* m_hull;
public:
	ConvexHullModifier( World* world, ConvexHull* hull );

	void SetMatrix( const dFloat* matrix );
	PyObject* GetMatrix();
};

class NullCollider : public CollisionGeometry
{
public: 
	NullCollider( World* world );
};


class TreeCollisionUserCallback 
{
public:
	virtual void OnCallback  ( Body* bodyWithTreeCollision, Body* body,
		const dFloat* vertex, int vertexstrideInBytes, 
		int indexCount, const int* indexArray); 
};

class TreeCollision : public CollisionGeometry
{
public:

public: 
	TreeCollision( World* world) ;


	void BeginBuild();
	void AddFace( PyObject* vertexList, int faceAttribute ) ;
	void EndBuild( int optimize);

	int  GetFaceAttribute ( const int* faceIndexArray ) ;
	void SetFaceAttribute( const int* faceIndexArray, int attribute ) ;

	bool Serialize( const char* filename );
	bool Deserialize( const char* filename );

};

//Not technically a newton collision, but it's useful and in the samples
/*class HeightmapCollision : public CollisionGeometry
{
public:
	HeightmapCollision( World* world, int size, dFloat cellsize
};*/

#endif
