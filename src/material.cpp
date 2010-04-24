#include "material.h"


Material::Material( const NewtonMaterial* m )
{
	m_material = (NewtonMaterial*)m;
}
Material::~Material()
{
}

void Material::DisableContact ()
{
	NewtonMaterialDisableContact( m_material );
}

dFloat Material::GetCurrentTimestep ()
{
	return NewtonMaterialGetCurrentTimestep( m_material );
}

/*void* Material::GetMaterialPairUserData ()
{
	return NULL;
}*/

unsigned Material::GetContactFaceAttribute ()
{
	return NewtonMaterialGetContactFaceAttribute( m_material );
}

unsigned Material::GetBodyCollisionID ( Body* body )
{
	return NewtonMaterialGetBodyCollisionID( m_material, body->m_body );
}

dFloat Material::GetContactNormalSpeed ( const NewtonContact* contactlHandle)
{
	return NewtonMaterialGetContactNormalSpeed( m_material, contactlHandle );
}


PyObject* Material::GetContactForce ( )
{
	dFloat v[3];
	NewtonMaterialGetContactForce( m_material, v );
	PyObject* r = PyTuple_New( 3 );
	PyTuple_SetItem( r, 0, PyFloat_FromDouble( v[0] ) );
	PyTuple_SetItem( r, 1, PyFloat_FromDouble( v[1] ) );
	PyTuple_SetItem( r, 2, PyFloat_FromDouble( v[2] ) );
	return r;
	
}

PyObject* Material::GetContactPosition ( )
{
	dFloat v[3];
	dFloat n[3];
	NewtonMaterialGetContactPositionAndNormal( m_material, v, n );
	PyObject* r = PyTuple_New( 3 );
	PyTuple_SetItem( r, 0, PyFloat_FromDouble( v[0] ) );
	PyTuple_SetItem( r, 1, PyFloat_FromDouble( v[1] ) );
	PyTuple_SetItem( r, 2, PyFloat_FromDouble( v[2] ) );
	return r;
}

PyObject* Material::GetContactNormal()
{
	dFloat v[3];
	dFloat n[3];
	NewtonMaterialGetContactPositionAndNormal( m_material, v, n );
	PyObject* r = PyTuple_New( 3 );
	PyTuple_SetItem( r, 0, PyFloat_FromDouble( n[0] ) );
	PyTuple_SetItem( r, 1, PyFloat_FromDouble( n[1] ) );
	PyTuple_SetItem( r, 2, PyFloat_FromDouble( n[2] ) );
	return r;
}

PyObject* Material::GetContactTangentDirections ( )
{
	dFloat d0[3];
	dFloat d1[3];
	NewtonMaterialGetContactTangentDirections( m_material, d0, d1 );
	PyObject* r = PyTuple_New( 6 );
	PyTuple_SetItem( r, 0, PyFloat_FromDouble( d0[0] ) );
	PyTuple_SetItem( r, 1, PyFloat_FromDouble( d0[1] ) );
	PyTuple_SetItem( r, 2, PyFloat_FromDouble( d0[2] ) );
	PyTuple_SetItem( r, 3, PyFloat_FromDouble( d1[0] ) );
	PyTuple_SetItem( r, 4, PyFloat_FromDouble( d1[1] ) );
	PyTuple_SetItem( r, 5, PyFloat_FromDouble( d1[2] ) );
	return r;
}

dFloat Material::GetContactTangentSpeed ( const NewtonContact* contactlHandle, int index)
{
	return NewtonMaterialGetContactTangentSpeed( m_material, contactlHandle, index );
}

void Material::SetContactSoftness	( dFloat softness)
{
	NewtonMaterialSetContactSoftness( m_material, softness );
}

void Material::SetContactElasticity( dFloat restitution)
{
	NewtonMaterialSetContactElasticity( m_material, restitution );
}

void Material::SetContactFrictionState( int state, int index)
{
	NewtonMaterialSetContactFrictionState( m_material, state, index );
}

void Material::SetContactStaticFrictionCoef  ( dFloat coef, int index)
{
	NewtonMaterialSetContactStaticFrictionCoef( m_material, coef, index );
}

void Material::SetContactKineticFrictionCoef ( dFloat coef, int index)
{
	NewtonMaterialSetContactKineticFrictionCoef( m_material, coef, index );
}


void Material::SetContactNormalAcceleration ( dFloat accel)
{
	NewtonMaterialSetContactNormalAcceleration( m_material, accel );
}

void Material::SetContactNormalDirection ( const dFloat* directionVector)
{
	NewtonMaterialSetContactNormalDirection( m_material, directionVector );
}


void Material::SetContactTangentAcceleration (dFloat accel, int index)
{
	NewtonMaterialSetContactTangentAcceleration( m_material, accel, index );
}

void Material::ContactRotateTangentDirections (const dFloat* directionVector)
{
	NewtonMaterialContactRotateTangentDirections( m_material, directionVector );
}

