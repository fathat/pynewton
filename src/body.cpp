#include <Python.h>
#include <Newton.h>
#include "wrapper.h"

static int SetPhysicsPlane (const int collisionID, void *context, const dFloat* globalSpaceMatrix, dFloat* globalSpacePlane)
{
	dFloat* plane = (dFloat*) context;
	globalSpacePlane[0] = plane[0];
	globalSpacePlane[1] = plane[1];
	globalSpacePlane[2] = plane[2];
	globalSpacePlane[3] = plane[3];
	return 1;	
}

void GeneralApplyForceAndTorqueCallback ( const NewtonBody* body ) 
{
	Body* wrapper = (Body*) NewtonBodyGetUserData( body );

	if(wrapper)
		wrapper->OnApplyForceAndTorque( );
}

void GeneralBodyDestruct ( const NewtonBody* body ) 
{
	Body* wrapper = (Body*) NewtonBodyGetUserData( body );

	if(wrapper)
		wrapper->OnDestruct();
}

void GeneralBodyLeaveWorldEvent( const NewtonBody* body )
{
	Body* wrapper = (Body*) NewtonBodyGetUserData( body );
	World* world = wrapper->GetWorld();
}


void GeneralAutoactiveCallback( const NewtonBody* body, unsigned state )
{
	Body* wrapper = (Body*) NewtonBodyGetUserData( body );
	wrapper->OnAutoactive( state );
}

void GeneralOnTransformedCallback( const NewtonBody* body, const dFloat* matrix)
{
	Body* wrapper = (Body*) NewtonBodyGetUserData( body );

	PyObject* pylist = PyList_New( 16 );
	Py_INCREF( pylist );
	for(int i=0; i<16; i++ )
		PyList_SetItem( pylist, i, PyFloat_FromDouble( matrix[i] )  );
	wrapper->OnTransform(  );
	Py_DECREF( pylist );


}



Body* Body::WrapperFor( NewtonBody* bodyptr ) {
	return new Body( bodyptr );
}

void Body::InitCallbacks( )
{
	NewtonBodySetUserData( m_body, this ) ;
	NewtonBodySetForceAndTorqueCallback( m_body, GeneralApplyForceAndTorqueCallback );
	NewtonBodySetAutoactiveCallback( m_body, GeneralAutoactiveCallback );
	NewtonBodySetDestructorCallback( m_body, GeneralBodyDestruct );
	NewtonBodySetTransformCallback( m_body, GeneralOnTransformedCallback );

}

Body::Body( const NewtonBody* body ) 
{
	m_cleanupOnDestruct = false; 
	m_zombie = false;
	m_world = new World( NewtonBodyGetWorld( body ) );
	m_body  = body;
	m_collision = NULL;

	InitCallbacks();

}

Body::Body( World* w, CollisionGeometry* g ) 
{
	m_cleanupOnDestruct = true;
	m_zombie = false; 
	m_world = w;
	m_body  = NewtonCreateBody( w->GetNewtonWorld(), g->GetNewtonCollision() ) ;
	m_collision = g;
	
	InitCallbacks();
}

Body::~Body( ) 
{
	if(m_cleanupOnDestruct )
	{
		PyObject* data =  GetUserData() ;
		Py_XDECREF( data );

		NewtonDestroyBody( m_world->GetNewtonWorld(), m_body ) ;
		m_body = NULL;

	}
	else
	{
		Py_XDECREF( userData);
		delete m_world;
	}
}
Body::Body ( const Body& body )
{
	m_cleanupOnDestruct = false; 
	m_zombie = false;
	m_world  = new World( NewtonBodyGetWorld( body.m_body ) );
	m_body   = body.m_body;
	m_collision = NULL;


	userData = body.userData;
	Py_XINCREF( userData);
}

void Body::operator = ( const Body& body )
{
	m_cleanupOnDestruct = false; 
	m_zombie = false;
	m_world  = new World( NewtonBodyGetWorld( body.m_body ) );
	m_body   = body.m_body;
	m_collision = NULL;
	
	Py_XINCREF( userData);
	userData = body.userData;


}

NewtonCollision* Body::GetNewtonCollision()
{
	return m_collision->GetNewtonCollision();
}

NewtonWorld* Body::GetNewtonWorld()
{
	return m_world->GetNewtonWorld();
}

int Body::IDKey()
{
	return (int)m_body;
}

void Body::SetMassMatrix( dFloat mass, dFloat ix, dFloat iy, dFloat iz ) 
{
	NewtonBodySetMassMatrix	( m_body, mass, ix, iy, iz );
}

void Body::SetMatrix( const dFloat* matrix )
{
	NewtonBodySetMatrix( m_body, matrix ) ;
}


void Body::SetMatrixRecursive (  const dFloat* matrix )
{
	NewtonBodySetMatrixRecursive( m_body, matrix );
}

void Body::SetOmega ( const dFloat* omega )
{
	NewtonBodySetOmega	( m_body, omega );
}

void Body::SetForce( const dFloat* force )
{
	NewtonBodySetForce( m_body, force );
}

void Body::SetTorque( const dFloat* torque )
{
	NewtonBodySetTorque( m_body, torque );
}


void Body::AddForce( const dFloat* force )
{
	NewtonBodyAddForce( m_body, force );
}

void Body::AddTorque( const dFloat* torque )
{
	NewtonBodyAddTorque( m_body, torque );
}


void  Body::SetVelocity ( const dFloat* velocity)
{
	NewtonBodySetVelocity( m_body, velocity );
}


PyObject* Body::GetMatrix( )
{

	dFloat matrix[16];
	NewtonBodyGetMatrix( m_body, matrix );
	PyObject* pylist = PyList_New( 16 );
	for(int i=0; i<16; i++ )
		PyList_SetItem( pylist, i, PyFloat_FromDouble( matrix[i] )  );
	return pylist;
}

PyObject* Body::GetMassMatrix()
{
	dFloat mass, ix, iy, iz;
	NewtonBodyGetMassMatrix( m_body, &mass, &ix, &iy, &iz );
	PyObject* pytuple = PyTuple_New( 4 );
	PyTuple_SetItem( pytuple, 0, PyFloat_FromDouble( mass ) );
	PyTuple_SetItem( pytuple, 1, PyFloat_FromDouble( ix ) );
	PyTuple_SetItem( pytuple, 2, PyFloat_FromDouble( iy ) );
	PyTuple_SetItem( pytuple, 3, PyFloat_FromDouble( iz ) );

	return pytuple;
}

PyObject* Body::GetForce( )
{
	dFloat v[3];
	NewtonBodyGetForce( m_body, v );
	PyObject* r = PyTuple_New( 3 );
	PyTuple_SetItem( r, 0, PyFloat_FromDouble( v[0] ) );
	PyTuple_SetItem( r, 1, PyFloat_FromDouble( v[1] ) );
	PyTuple_SetItem( r, 2, PyFloat_FromDouble( v[2] ) );
	return r;
}

PyObject* Body::GetOmega( )
{
	dFloat v[3];
	NewtonBodyGetOmega( m_body, v );
	PyObject* r = PyTuple_New( 3 );
	PyTuple_SetItem( r, 0, PyFloat_FromDouble( v[0] ) );
	PyTuple_SetItem( r, 1, PyFloat_FromDouble( v[1] ) );
	PyTuple_SetItem( r, 2, PyFloat_FromDouble( v[2] ) );
	return r;
}

PyObject* Body::GetVelocity( )
{
	dFloat v[3];
	NewtonBodyGetVelocity( m_body, v );
	PyObject* r = PyTuple_New( 3 );
	PyTuple_SetItem( r, 0, PyFloat_FromDouble( v[0] ) );
	PyTuple_SetItem( r, 1, PyFloat_FromDouble( v[1] ) );
	PyTuple_SetItem( r, 2, PyFloat_FromDouble( v[2] ) );
	return r;

}

PyObject* Body::GetTorque ( )
{
	dFloat v[3];
	NewtonBodyGetTorque( m_body, v );
	PyObject* r = PyTuple_New( 3 );
	PyTuple_SetItem( r, 0, PyFloat_FromDouble( v[0] ) );
	PyTuple_SetItem( r, 1, PyFloat_FromDouble( v[1] ) );
	PyTuple_SetItem( r, 2, PyFloat_FromDouble( v[2] ) );
	return r;
}

PyObject* Body::GetCentreOfMass ()
{
	dFloat v[3];
	NewtonBodyGetCentreOfMass( m_body, v );
	PyObject* r = PyTuple_New( 3 );
	PyTuple_SetItem( r, 0, PyFloat_FromDouble( v[0] ) );
	PyTuple_SetItem( r, 1, PyFloat_FromDouble( v[1] ) );
	PyTuple_SetItem( r, 2, PyFloat_FromDouble( v[2] ) );
	return r;
}

void Body::Freeze()
{
	NewtonWorldFreezeBody( m_world->GetNewtonWorld(), m_body );
}

void Body::Unfreeze()
{
	NewtonWorldUnfreezeBody( m_world->GetNewtonWorld(), m_body );
}

void  Body::SetCentreOfMass (const dFloat* vector)
{
	NewtonBodySetCentreOfMass( m_body, vector );
}

void  Body::SetLinearDamping (dFloat linearDamp)
{
	NewtonBodySetLinearDamping( m_body, linearDamp );
}

void  Body::SetAngularDamping (const dFloat* vector)
{
	NewtonBodySetAngularDamping( m_body, vector );
}

void  Body::CoriolisForcesMode (int mode)
{
	NewtonBodyCoriolisForcesMode ( m_body, mode );
}

void  Body::SetCollision (CollisionGeometry* collision)
{
	NewtonBodySetCollision( m_body, collision->GetNewtonCollision());
}

void  Body::SetAutoFreeze (int state)
{
	NewtonBodySetAutoFreeze( m_body, state );
}

void  Body::SetFreezeTreshold  (dFloat freezeSpeed2, dFloat freezeOmega2, int framesCount)
{
	NewtonBodySetFreezeTreshold( m_body, freezeSpeed2, freezeOmega2, framesCount );
}

int Body::GetSleepingState ()
{
	return NewtonBodyGetSleepingState( m_body );
}
int Body::GetAutoFreeze    ()
{
	return NewtonBodyGetAutoFreeze( m_body );
}

dFloat Body::GetLinearDamping( )
{
	return NewtonBodyGetLinearDamping(m_body);
}

//returns (x,y,z) tuple
PyObject* Body::GetAngularDamping ()
{
	dFloat v[3];
	NewtonBodyGetAngularDamping( m_body, v );
	PyObject* r = PyTuple_New( 3 );
	PyTuple_SetItem( r, 0, PyFloat_FromDouble( v[0] ) );
	PyTuple_SetItem( r, 1, PyFloat_FromDouble( v[1] ) );
	PyTuple_SetItem( r, 2, PyFloat_FromDouble( v[2] ) );
	return r;
}

//returns list of 6 numbers [x1, y1, z1, x2, y2, z2] defining AABB bounds
PyObject* Body::GetAABB ( )
{
	dFloat v[3];
	dFloat v2[3];
	NewtonBodyGetAABB( m_body, v, v2 );
	PyObject* r = PyTuple_New( 6 );
	PyTuple_SetItem( r, 0, PyFloat_FromDouble( v[0] ) );
	PyTuple_SetItem( r, 1, PyFloat_FromDouble( v[1] ) );
	PyTuple_SetItem( r, 2, PyFloat_FromDouble( v[2] ) );
	PyTuple_SetItem( r, 3, PyFloat_FromDouble( v2[0] ) );
	PyTuple_SetItem( r, 4, PyFloat_FromDouble( v2[1] ) );
	PyTuple_SetItem( r, 5, PyFloat_FromDouble( v2[2] ) );
	
	return r;

}

//returns tuple of (freezeSpeed, freezeOmega )
PyObject* Body::GetFreezeTreshold( )
{
	dFloat s;
	dFloat o;
	NewtonBodyGetFreezeTreshold( m_body, &s, &o );
	PyObject* r = PyTuple_New( 2 );
	PyTuple_SetItem( r, 0, PyFloat_FromDouble( s ) );
	PyTuple_SetItem( r, 1, PyFloat_FromDouble( o ) );
	return r;
}

void  Body::SetMaterialGroupID (  int id)
{
	NewtonBodySetMaterialGroupID( m_body, id );
}

void  Body::SetContinuousCollisionMode ( unsigned state)
{
	NewtonBodySetContinuousCollisionMode( m_body, state );
}

void  Body::SetJointRecursiveCollision (unsigned state)
{
	NewtonBodySetJointRecursiveCollision( m_body, state );
}

void  Body::SetDestructorCallback ( NewtonBodyDestructor callback)
{
	NewtonBodySetDestructorCallback( m_body, callback );
}

PyObject* Body::GetUserData ()
{
	return userData;
}

void Body::SetUserData( PyObject* data )
{
	Py_XDECREF( userData);
	Py_XINCREF( data );
	userData = data;
}



int   Body::MaterialGroupID ()
{
	return NewtonBodyGetMaterialGroupID(m_body );
}

int   Body::GetContinuousCollisionMode ()
{
	return NewtonBodyGetContinuousCollisionMode( m_body );
}

int   Body::GetJointRecursiveCollision ()
{
	return NewtonBodyGetJointRecursiveCollision( m_body );
}


//returns 4 values
PyObject*  Body::GetInvMass( )
{
	dFloat mass, x, y, z;
	NewtonBodyGetInvMass( m_body, &mass, &x, &y, &z ) ;
	PyObject* r = PyTuple_New( 4);
	PyTuple_SetItem( r, 0, PyFloat_FromDouble( mass ) );
	PyTuple_SetItem( r, 1, PyFloat_FromDouble( x ) );
	PyTuple_SetItem( r, 2, PyFloat_FromDouble( y ) );
	PyTuple_SetItem( r, 3, PyFloat_FromDouble( z ) );
	return r;
	
}


void  Body::AddBuoyancyForce (dFloat fluidDensity, 
						dFloat fluidLinearViscosity,
						dFloat fluidAngularViscosity, 
						const dFloat* gravityVector, 
						const dFloat* plane)
{
	dFloat* context = new dFloat[4];
	context[0] = plane[0];
	context[1] = plane[1];
	context[2] = plane[2];
	context[3] = plane[3];
	NewtonBodyAddBuoyancyForce( m_body, 
		fluidDensity, 
		fluidLinearViscosity, 
		fluidAngularViscosity, 
		gravityVector, 
		SetPhysicsPlane, 
		context );
}

void Body::ForEachPolygonDo ( NewtonCollisionIterator callback)
{

}

void Body::AddBodyImpulse ( const dFloat* velocity, const dFloat* point)
{
	NewtonAddBodyImpulse( m_body, velocity, point );

}

void Body::OnApplyForceAndTorque( ) 
{

}


void Body::OnAutoactive(unsigned int state) 
{
}

void Body::OnTransform( )
{
	
}

void Body::OnDestruct( )
{
}

void Body::OnTreeCollisionWith( Body* body)
{
	
}
