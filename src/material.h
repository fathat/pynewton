#ifndef MATERIAL_H
#define MATERIAL_H

#include <Python.h>
#include <Newton.h>
#include "body.h"

class Material
{
public:
	NewtonMaterial* m_material;

public:
	Material( const NewtonMaterial* m );
	~Material();

	void DisableContact ();
	dFloat GetCurrentTimestep ();
	//PyObject* GetMaterialPairUserData ();
	unsigned GetContactFaceAttribute ();
	unsigned GetBodyCollisionID ( Body* body );
	dFloat GetContactNormalSpeed ( const NewtonContact* contactlHandle);

	//returns an (x,y,z) tuple
	PyObject* GetContactForce ();
	PyObject* GetContactPosition();
	PyObject* GetContactNormal();

	//returns (x1, y1, z1, x2, y2, z2 )
	PyObject* GetContactTangentDirections ();

	dFloat GetContactTangentSpeed ( const NewtonContact* contactlHandle, int index);

	void SetContactSoftness	( dFloat softness);
	void SetContactElasticity( dFloat restitution);
	void SetContactFrictionState( int state, int index);
	void SetContactStaticFrictionCoef  ( dFloat coef, int index);
	void SetContactKineticFrictionCoef ( dFloat coef, int index);

	void SetContactNormalAcceleration ( dFloat accel);
	void SetContactNormalDirection ( const dFloat* directionVector);

	void SetContactTangentAcceleration (dFloat accel, int index);
	void ContactRotateTangentDirections (const dFloat* directionVector);

};

#endif
