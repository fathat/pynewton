#pragma once

#include <vector>
#include <Python.h>
#include <Newton.h>
#include "world.h"

class Joint 
{
public:
	World* m_world;
	NewtonJoint* m_joint;

	PyObject* m_userData;

public:
	Joint();
	virtual ~Joint();

	void ForceDestroy();

	void SetUserData( PyObject* userData );
	PyObject* GetUserData();

	
	void SetCollisionState ( int state );
	int  GetCollisionState() ;

	void SetStiffness( dFloat stiffness );
	dFloat GetStiffness( );

	virtual void OnDestroyed( ) ;
		
};


class BallJoint : public Joint 
{
public:
	BallJoint( World* world, dFloat* pivotPoint, Body* childBody, Body* parentBody );

	PyObject* GetJointAngle( );
	PyObject* GetJointOmega( );
	PyObject* GetJointForce( );

	virtual void OnCallback( ) ;

	void SetConeLimits( const dFloat* pin, dFloat maxConeAngle, dFloat maxTwistAngle );

};


class Hinge : public Joint
{
public: 
	Hinge( World* world, const dFloat* pivotPoint, const dFloat* pinDir, Body* childBody, Body* parentBody );

	dFloat GetJointAngle();
	dFloat GetJointOmega();
	PyObject* GetJointForce();

	dFloat CalculateStopAlpha( NewtonHingeSliderUpdateDesc* desc, dFloat angle );

	virtual unsigned OnCallback( NewtonHingeSliderUpdateDesc* desc );
	
};


class Slider : public Joint
{
	dFloat m_minLimit, m_maxLimit;
	bool m_useLimits;
public:
	Slider( World* world, const dFloat* pivotPoint, const dFloat* pinDir, Body* childBody, Body* parentBody  );

	dFloat GetPosition();
	dFloat GetVelocity();
	PyObject* GetForce();

	void SetMinLimit( dFloat f );
	void SetMaxLimit( dFloat f );
	dFloat GetMinLimit();
	dFloat GetMaxLimit();

	dFloat CalculateStopAccel ( NewtonHingeSliderUpdateDesc* desc, dFloat position );

	virtual unsigned OnCallback( NewtonHingeSliderUpdateDesc* desc );
	
};


class Corkscrew : public Joint
{
public:
	Corkscrew( World* world, const dFloat* pivotPoint, const dFloat* pinDir, Body* childBody, Body* parentBody );

	dFloat GetPosition( );
	dFloat GetAngle( );
	dFloat GetVelocity();
	dFloat GetOmega ( );
	PyObject* GetForce();

	virtual unsigned OnCallback( NewtonHingeSliderUpdateDesc* desc );

	dFloat CalculateStopAlpha(  const NewtonHingeSliderUpdateDesc* desc, dFloat angle);
	dFloat CalculateStopAccel (const NewtonJoint* corkscrew, const NewtonHingeSliderUpdateDesc* desc, dFloat position);
};

class UniversalJoint : public Joint
{
public:
	UniversalJoint ( World* world, 
		const dFloat* pivotPoint, const dFloat* pinDir0, const dFloat* pinDir1, 
		Body* childBody, Body* parentBody);


	dFloat GetAngle0 ();
	dFloat GetAngle1 ();
	dFloat GetOmega0 ();
	dFloat GetOmega1 ();
	PyObject* GetForce();

	virtual unsigned OnCallback( NewtonHingeSliderUpdateDesc* desc );

	dFloat CalculateStopAlpha0 ( const NewtonHingeSliderUpdateDesc* desc, dFloat angle );
	dFloat CalculateStopAlpha1 ( const NewtonHingeSliderUpdateDesc* desc, dFloat angle );
	
};


class UpVector : public Joint
{
public:
	UpVector (World* world, const dFloat* pinDir, Body* body);
	PyObject* GetPin ( );
	void SetPin ( const dFloat* pin );

};

class UserJoint : public Joint
{
	/*NEWTON_API NewtonJoint* NewtonConstraintCreateUserJoint (const NewtonWorld* newtonWorld, int maxDOF, NewtonUserBilateralCallBack callback,
		const NewtonBody* childBody, const NewtonBody* parentBody); 

	NEWTON_API void NewtonUserJointAddLinearRow (const NewtonJoint* joint, const dFloat *pivot0, const dFloat *pivot1, const dFloat *dir);
	NEWTON_API void NewtonUserJointAddAngularRow (const NewtonJoint* joint, dFloat relativeAngle, const dFloat *dir);
	NEWTON_API void NewtonUserJointAddGeneralRow (const NewtonJoint* joint, const dFloat *jacobian0, const dFloat *jacobian1);
	NEWTON_API void NewtonUserJointSetRowMinimumFriction (const NewtonJoint* joint, dFloat friction);
	NEWTON_API void NewtonUserJointSetRowMaximumFriction (const NewtonJoint* joint, dFloat friction);
	NEWTON_API void NewtonUserJointSetRowAcceleration (const NewtonJoint* joint, dFloat acceleration);
	NEWTON_API void NewtonUserJointSetRowSpringDamperAcceleration (const NewtonJoint* joint, dFloat springK, dFloat springD);
	NEWTON_API void NewtonUserJointSetRowStiffness (const NewtonJoint* joint, dFloat stiffness);
	NEWTON_API dFloat NewtonUserJointGetRowForce (const NewtonJoint* joint, int row);*/

};

class Vehicle;

class Tire 
{
public:
	Vehicle* m_vehicle;
	void* m_tireID;
	PyObject* m_userData;

public:

	Tire( Vehicle* v,  void* tireid ) ;
	Tire( const Tire& t ); //copy constructor
	~Tire();

	Tire& operator = ( Tire& t );

	void SetUserData( PyObject* userData);
	PyObject* GetUserData( );

	bool IsAirBorne();
	bool LostSideGrip();
	bool LostTraction();

	dFloat GetOmega ();
	dFloat GetNormalLoad ();
	dFloat GetSteerAngle ();
	dFloat GetLateralSpeed ();
	dFloat GetLongitudinalSpeed ();

	

	PyObject* GetMatrix ();


	void SetTorque ( dFloat torque);
	void SetSteerAngle ( dFloat angle);

	void SetMaxSideSleepSpeed ( dFloat speed);
	void SetSideSleepCoeficient (dFloat coeficient);
	void SetMaxLongitudinalSlideSpeed (dFloat speed);
	void SetLongitudinalSlideCoeficient (dFloat coeficient);

	dFloat CalculateMaxBrakeAcceleration ();
	void SetBrakeAcceleration ( dFloat accel, dFloat torqueLimit);

};

class Vehicle : public Joint
{
	std::vector<Tire*> tireList;
public:
	Vehicle( World* world, const dFloat* upDir, Body* body);
	~Vehicle();
	
	void Reset();

	Tire* AddTire( const dFloat* matrix, const dFloat* pin, dFloat mass, dFloat width, dFloat radius, 
		    dFloat suspensionShock, dFloat suspensionSpring, dFloat suspensionLength, PyObject* userData, int collisionID);
	void RemoveTire( Tire* tire );

	virtual void OnCallback( );

	Tire* GetFirstTire( );
	Tire* GetNextTireID( Tire* tireID );
	

/*	 void NewtonVehicleSetTireCallback (const NewtonJoint* vehicle, NewtonVehicleTireUpdate update);
 void* NewtonVehicleAddTire (const NewtonJoint* vehicle, const dFloat* localMatrix, const dFloat* pin, dFloat mass, dFloat width, dFloat radius, 
		    dFloat suspesionShock, dFloat suspesionSpring, dFloat suspesionLength, void* userData, int collisionID);
 void NewtonVehicleRemoveTire (const NewtonJoint* vehicle, void* tireId);

 void* NewtonVehicleGetFirstTireID (const NewtonJoint* vehicle);
 void* NewtonVehicleGetNextTireID (const NewtonJoint* vehicle, void* tireId);

 int NewtonVehicleTireIsAirBorne (const NewtonJoint* vehicle, void* tireId);
 int NewtonVehicleTireLostSideGrip (const NewtonJoint* vehicle, void* tireId);
 int NewtonVehicleTireLostTraction (const NewtonJoint* vehicle, void* tireId);

 void* NewtonVehicleGetTireUserData (const NewtonJoint* vehicle, void* tireId);
 dFloat NewtonVehicleGetTireOmega (const NewtonJoint* vehicle, void* tireId);
 dFloat NewtonVehicleGetTireNormalLoad (const NewtonJoint* vehicle, void* tireId);
 dFloat NewtonVehicleGetTireSteerAngle (const NewtonJoint* vehicle, void* tireId);
 dFloat NewtonVehicleGetTireLateralSpeed (const NewtonJoint* vehicle, void* tireId);
 dFloat NewtonVehicleGetTireLongitudinalSpeed (const NewtonJoint* vehicle, void* tireId);
 void NewtonVehicleGetTireMatrix (const NewtonJoint* vehicle, void* tireId, dFloat* matrix);


 void NewtonVehicleSetTireTorque (const NewtonJoint* vehicle, void* tireId, dFloat torque);
 void NewtonVehicleSetTireSteerAngle (const NewtonJoint* vehicle, void* tireId, dFloat angle);

 void NewtonVehicleSetTireMaxSideSleepSpeed (const NewtonJoint* vehicle, void* tireId, dFloat speed);
 void NewtonVehicleSetTireSideSleepCoeficient (const NewtonJoint* vehicle, void* tireId, dFloat coeficient);
 void NewtonVehicleSetTireMaxLongitudinalSlideSpeed (const NewtonJoint* vehicle, void* tireId, dFloat speed);
 void NewtonVehicleSetTireLongitudinalSlideCoeficient (const NewtonJoint* vehicle, void* tireId, dFloat coeficient);

 dFloat NewtonVehicleTireCalculateMaxBrakeAcceleration (const NewtonJoint* vehicle, void* tireId);
 void NewtonVehicleTireSetBrakeAcceleration (const NewtonJoint* vehicle, void* tireId, dFloat accelaration, dFloat torqueLimit);*/


};

