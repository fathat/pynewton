#include <Python.h>
#include <Newton.h>
#include "world.h"
#include "body.h"
#include "material.h"
#include <iostream>
using namespace std;

//Collision Detection
dFloat  OnRayCastPlacement (const NewtonBody* body, const dFloat* normal, int collisionID, void* userData, dFloat intersectParam)
{
	World* world = (World*)userData;
	return world->RayCastCallback( body, normal[0], normal[1], normal[2], collisionID, intersectParam );
	
}

World::World( NewtonWorld* world ) 
{
	m_world = world;
	ownsWorld = false;
}

World::World( )
{
	m_world = NewtonCreate( NULL, NULL );
	ownsWorld = true;
}

World::~World()
{
	if(ownsWorld )
	{
		NewtonDestroy( m_world );
		m_world = NULL;

	}
}

void World::RegisterBody( Body* body )
{

}

void World::UnregisterBody( Body* body )
{
}


NewtonWorld* World::GetNewtonWorld()
{
	return m_world ;
}

void World::Update(dFloat timeStep )
{
	NewtonUpdate( m_world, timeStep );
}

void World::SetPlatformArchitecture ( int mode )
{
	NewtonSetPlatformArchitecture( m_world, mode );
}

void World::SetSolverModel ( int model )
{
	NewtonSetSolverModel( m_world, model );
}

void World::SetFrictionModel ( int model )
{
	NewtonSetFrictionModel( m_world, model );
}

dFloat World::GetTimeStep ()
{
	return NewtonGetTimeStep( m_world );
}

void World::SetMinimumFrameRate ( dFloat frameRate)
{
	NewtonSetMinimumFrameRate ( m_world, frameRate );
}


void World::SetWorldSize ( const dFloat* minPoint, const dFloat* maxPoint )
{
	NewtonSetWorldSize( m_world, minPoint, maxPoint );
}

int World::GetVersion()
{
	return NewtonWorldGetVersion( m_world );
}

void World::RemoveBody( Body* body )
{
	NewtonDestroyBody( m_world, body->m_body );
}


void World::SetUserData( PyObject* data )
{
	Py_XDECREF( GetUserData() );

	Py_INCREF( data );
	NewtonWorldSetUserData( m_world, data );
}

PyObject* World::GetUserData()
{
	return (PyObject*) NewtonWorldGetUserData( m_world );
}

void World::CppRayCast( float ax, float ay, float az, float bx, float by, float bz)
{
	dFloat a[3] = { ax, ay, az };
	dFloat b[3] = { bx, by, bz };
	NewtonWorldRayCast( this->m_world, a, b, OnRayCastPlacement, this, NULL);
}

dFloat World::RayCastCallback (const NewtonBody* body, dFloat nx, dFloat ny, dFloat nz, int collisionID, dFloat intersectParam)
{
	return 1.2f;
}



//Material stuff
int  World::MaterialGetDefaultGroupID()
{
	return NewtonMaterialGetDefaultGroupID(m_world);
}

int  World::MaterialCreateGroupID()
{
	return NewtonMaterialCreateGroupID( m_world );
}

void World::MaterialDestroyAllGroupID()
{
	NewtonMaterialDestroyAllGroupID( m_world );
}

void World::MaterialSetDefaultSoftness   ( int id0, int id1, dFloat value)
{
	NewtonMaterialSetDefaultSoftness( m_world, id0, id1, value );
}

void World::MaterialSetDefaultElasticity ( int id0, int id1, dFloat elasticCoef)
{
	NewtonMaterialSetDefaultElasticity( m_world, id0, id1, elasticCoef );
}

void World::MaterialSetDefaultCollidable ( int id0, int id1, int state)
{
	NewtonMaterialSetDefaultCollidable( m_world, id0, id1, state );
}

void World::MaterialSetContinuousCollisionMode ( int id0, int id1, int state)
{
	NewtonMaterialSetContinuousCollisionMode( m_world, id0, id1, state );
}

void World::MaterialSetDefaultFriction ( int id0, int id1, dFloat staticFriction, dFloat kineticFriction)
{
	NewtonMaterialSetDefaultFriction( m_world, id0, id1, staticFriction, kineticFriction );
}


int GenericNewtonContactBeginCallback( const NewtonMaterial* m, const NewtonBody* b1, const NewtonBody* b2 )
{
	World* world = (World*)NewtonMaterialGetMaterialPairUserData( m ); 
	world->MaterialBeginCollision( Material( m ), b1, b2 );
	return 1;
}

int GenericNewtonContactProcessCallback( const NewtonMaterial* m, const NewtonContact* c )
{
	World* world = (World*)NewtonMaterialGetMaterialPairUserData( m ); 
	world->MaterialProcessCollision( Material( m ), c );
	return 1;
}

void GenericNewtonContactEndCallback( const NewtonMaterial* m )
{
	World* world = (World*)NewtonMaterialGetMaterialPairUserData( m ); 
	world->MaterialEndCollision( Material(m) );
}

void World::MaterialBeginCollision(  Material mat, const NewtonBody* b1, const NewtonBody* b2 )
{
	
}

void World::MaterialProcessCollision( Material mat, const NewtonContact* contactHandle )
{
}

void World::MaterialEndCollision( Material mat )
{
}

void World::RegisterMaterialCallbackBetween ( int id0, int id1 ) 
{
	NewtonMaterialSetCollisionCallback( m_world, id0, id1, this, 
				GenericNewtonContactBeginCallback, 
				GenericNewtonContactProcessCallback, 
				GenericNewtonContactEndCallback );
}

