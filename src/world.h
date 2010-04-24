#ifndef WORLD_H
#define WORLD_H

#include <Python.h>
#include "materialgroup.h"

class Body; 


class World
{
	NewtonWorld* m_world;
	bool ownsWorld;


public: 	

	World( NewtonWorld* world );
	World();
	virtual ~World();

	virtual void RegisterBody( Body* body );
	virtual void UnregisterBody( Body* body );

	NewtonWorld* GetNewtonWorld();

	void Update( dFloat timeStep );

	void SetPlatformArchitecture ( int mode );
	void SetSolverModel ( int model );
	void SetFrictionModel ( int model );
	dFloat GetTimeStep ();
	void SetMinimumFrameRate ( dFloat frameRate);

	void SetWorldSize ( const dFloat* minPoint, const dFloat* maxPoint );

	int GetVersion();

	void RemoveBody( Body* body );

	void SetUserData( PyObject* data );
	PyObject* GetUserData();

	//Collision stuff
	virtual dFloat RayCastCallback (const NewtonBody* body, dFloat nx, dFloat ny, dFloat nz, int collisionID, dFloat intersectParam);
	virtual void CppRayCast( float ax, float ay, float az, 
					 float bx, float by, float bz);

	//Material stuff
	int  MaterialGetDefaultGroupID();
	int  MaterialCreateGroupID();
	void MaterialDestroyAllGroupID();

	void MaterialSetDefaultSoftness   ( int id0, int id1, dFloat value);
	void MaterialSetDefaultElasticity ( int id0, int id1, dFloat elasticCoef);
	void MaterialSetDefaultCollidable ( int id0, int id1, int state);
	void MaterialSetContinuousCollisionMode ( int id0, int id1, int state);
	void MaterialSetDefaultFriction ( int id0, int id1, dFloat staticFriction, dFloat kineticFriction);

	virtual void MaterialBeginCollision( Material mat,  const NewtonBody* b1, const NewtonBody* b2  );
	virtual void MaterialProcessCollision(  Material mat, const NewtonContact* contactHandle  );
	virtual void MaterialEndCollision(  Material mat );

	void RegisterMaterialCallbackBetween ( int id0, int id1 );


};

#endif
