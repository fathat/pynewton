#ifndef BODY_H 
#define BODY_H

#include "world.h"
#include "collision.h"

class Body
{
private:
	void InitCallbacks();

public:
	const NewtonBody* m_body;
	World* m_world;
	CollisionGeometry* m_collision; 

	PyObject* userData;

	bool m_cleanupOnDestruct;
	bool m_zombie;

public:

	Body( const NewtonBody* body ) ;
	Body( World* w, CollisionGeometry* g ) ;
	Body( const Body& body );
	~Body();

	void operator = ( const Body& body );

	NewtonCollision* GetNewtonCollision();
	NewtonWorld* GetNewtonWorld();

	int IDKey();

	World* GetWorld() { return m_world ; }
	
	void SetMassMatrix( dFloat mass, dFloat ix, dFloat iy, dFloat iz ) ;
	void SetMatrix( const dFloat* matrix );
	void SetMatrixRecursive (  const dFloat* matrix );
	PyObject* GetMatrix( ); //returns a list for a matrix 
	PyObject* GetMassMatrix();


	void SetOmega ( const dFloat* omega );
	void SetForce( const dFloat* force );
	void SetTorque( const dFloat* torque );
	void AddForce( const dFloat* force );
	void AddTorque( const dFloat* torque );

	PyObject* GetForce( );
	PyObject* GetOmega( );
	PyObject* GetVelocity( );
	PyObject* GetTorque ( );
	PyObject* GetCentreOfMass ();

	void  SetVelocity ( const dFloat* velocity);

	void Freeze();
	void Unfreeze();

	void  SetCentreOfMass    (const dFloat* vector);
	void  SetLinearDamping   (dFloat linearDamp);
	void  SetAngularDamping  (const dFloat* vector);
	void  CoriolisForcesMode (int mode);
	void  SetCollision		 (CollisionGeometry* collision);
	void  SetAutoFreeze		 (int state);
	void  SetFreezeTreshold  (dFloat freezeSpeed2, dFloat freezeOmega2, int framesCount);

	int    GetSleepingState ();
	int    GetAutoFreeze    ();
	dFloat GetLinearDamping ();

	//returns (x,y,z) tuple
	PyObject* GetAngularDamping();

	//returns tuple of 6 numbers (x1, y1, z1, x2, y2, z2) defining AABB bounds
	PyObject* GetAABB ( );	

	//returns tuple of (freezeSpeed, freezeOmega )
	PyObject* GetFreezeTreshold( );


	//materials and stuff
	void  SetMaterialGroupID ( int id);
	void  SetContinuousCollisionMode ( unsigned state);
	void  SetJointRecursiveCollision (unsigned state);

	void  SetDestructorCallback ( NewtonBodyDestructor callback);

	void SetUserData( PyObject* data );
	PyObject* GetUserData ();

	
	virtual int   MaterialGroupID ();
	int   GetContinuousCollisionMode ();
	int   GetJointRecursiveCollision ();


	//Returns a tuple of (mass, Ixx, Iyy, Izz)
	PyObject* GetInvMass();


	void  AddBuoyancyForce (dFloat fluidDensity, 
						dFloat fluidLinearViscosity,
						dFloat fluidAngularViscosity, 
						const dFloat* gravityVector, 
						const dFloat* plane);

	void ForEachPolygonDo ( NewtonCollisionIterator callback);
	void AddBodyImpulse ( const dFloat* velocity, const dFloat* point);


	virtual void OnTreeCollisionWith( Body* body); 
	
	virtual void OnApplyForceAndTorque( );
	virtual void OnAutoactive(unsigned int state) ;
	virtual void OnTransform(  );
	virtual void OnDestruct( );

	static Body* WrapperFor( NewtonBody* bodyptr );


};

#endif

