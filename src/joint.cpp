#include "joint.h"
#include "world.h"
#include "body.h"
#include <algorithm>
#include <iostream>


void GeneralTireUpdateCallback (const NewtonJoint* vehicle)
{
	Vehicle* v = (Vehicle*)NewtonJointGetUserData( vehicle );

	if( v )
		v->OnCallback(  );
}

Joint::Joint()
{
	m_world = NULL;
	m_joint = NULL;
	m_userData = NULL;
}

Joint::~Joint()
{
	Py_XDECREF( m_userData );
	ForceDestroy();
	
}

void Joint::ForceDestroy()
{
	if(m_joint)
		NewtonDestroyJoint( m_world->GetNewtonWorld(), m_joint );
	m_joint = NULL;
}

void Joint::SetUserData( PyObject* userData )
{
	Py_XDECREF( m_userData );
	m_userData = userData;
	Py_XINCREF( m_userData );
}

PyObject* Joint::GetUserData()
{
	return m_userData;
}

void Joint::SetCollisionState ( int state )
{
	NewtonJointSetCollisionState( m_joint, state );
}

int  Joint::GetCollisionState() 
{
	return NewtonJointGetCollisionState( m_joint );
}

void Joint::SetStiffness( dFloat stiffness )
{
	NewtonJointSetStiffness( m_joint, stiffness );
}

dFloat Joint::GetStiffness( )
{
	return NewtonJointGetStiffness( m_joint );
}

void Joint::OnDestroyed( ) 
{
}


BallJoint::BallJoint( World* world, dFloat* pivotPoint, Body* childBody, Body* parentBody )
{
	m_world = world;
	m_joint = NewtonConstraintCreateBall( m_world->GetNewtonWorld(), pivotPoint, childBody->m_body, parentBody->m_body );
}

void BallJoint::OnCallback( ) 
{
}

PyObject* BallJoint::GetJointAngle( )
{
	dFloat v[3];
	NewtonBallGetJointAngle( m_joint, v );
	PyObject* r = PyTuple_New( 3 );
	PyTuple_SetItem( r, 0, PyFloat_FromDouble( v[0] ) );
	PyTuple_SetItem( r, 1, PyFloat_FromDouble( v[1] ) );
	PyTuple_SetItem( r, 2, PyFloat_FromDouble( v[2] ) );
	return r;
}

PyObject* BallJoint::GetJointOmega( )
{
	dFloat v[3];
	NewtonBallGetJointOmega( m_joint, v );
	PyObject* r = PyTuple_New( 3 );
	PyTuple_SetItem( r, 0, PyFloat_FromDouble( v[0] ) );
	PyTuple_SetItem( r, 1, PyFloat_FromDouble( v[1] ) );
	PyTuple_SetItem( r, 2, PyFloat_FromDouble( v[2] ) );
	return r;
}

PyObject* BallJoint::GetJointForce( )
{
	dFloat v[3];
	NewtonBallGetJointForce( m_joint, v );
	PyObject* r = PyTuple_New( 3 );
	PyTuple_SetItem( r, 0, PyFloat_FromDouble( v[0] ) );
	PyTuple_SetItem( r, 1, PyFloat_FromDouble( v[1] ) );
	PyTuple_SetItem( r, 2, PyFloat_FromDouble( v[2] ) );
	return r;
}

void BallJoint::SetConeLimits( const dFloat* pin, dFloat maxConeAngle, dFloat maxTwistAngle )
{
	NewtonBallSetConeLimits( m_joint, pin, maxConeAngle, maxTwistAngle );
}

Hinge::Hinge( World* world, const dFloat* pivotPoint, const dFloat* pinDir, Body* childBody, Body* parentBody )
{
	m_world = world;
	m_joint = NewtonConstraintCreateHinge( m_world->GetNewtonWorld(), pivotPoint, pinDir, childBody->m_body, parentBody->m_body );
}

dFloat Hinge::GetJointAngle()
{
    return NewtonHingeGetJointAngle( m_joint );	
}

dFloat Hinge::GetJointOmega()
{
	return NewtonHingeGetJointOmega( m_joint );
}

PyObject* Hinge::GetJointForce()
{
	dFloat v[3];
	NewtonHingeGetJointForce( m_joint, v );
	PyObject* r = PyTuple_New( 3 );
	PyTuple_SetItem( r, 0, PyFloat_FromDouble( v[0] ) );
	PyTuple_SetItem( r, 1, PyFloat_FromDouble( v[1] ) );
	PyTuple_SetItem( r, 2, PyFloat_FromDouble( v[2] ) );
	return r;
}


dFloat Hinge::CalculateStopAlpha( NewtonHingeSliderUpdateDesc* desc, dFloat angle )
{
	return NewtonHingeCalculateStopAlpha ( m_joint, desc, angle );
}

unsigned Hinge::OnCallback( NewtonHingeSliderUpdateDesc* desc )
{
	return 0;
}

static unsigned GenericSliderCallback (const NewtonJoint* slider, NewtonHingeSliderUpdateDesc* desc)
{
	Slider* s = (Slider*)NewtonJointGetUserData(slider);
	return s->OnCallback(desc);

}

Slider::Slider( World* world, const dFloat* pivotPoint, const dFloat* pinDir, Body* childBody, Body* parentBody  )
{
	m_world = world;
	m_useLimits = false;
	m_minLimit = 0;
	m_maxLimit = 0;
	m_joint = NewtonConstraintCreateSlider( m_world->GetNewtonWorld(), pivotPoint, pinDir, childBody->m_body, parentBody->m_body );
	NewtonJointSetUserData(m_joint,  this );
	NewtonSliderSetUserCallback( m_joint, GenericSliderCallback);
}

void Slider::SetMinLimit( dFloat f )
{
	m_useLimits = true;
	m_minLimit = f;
}

void Slider::SetMaxLimit( dFloat f )
{
	m_useLimits = true;
	m_maxLimit = f;
}

dFloat Slider::GetMinLimit()
{
	return m_minLimit;
}

dFloat Slider::GetMaxLimit()
{
	return m_maxLimit;
}

dFloat Slider::GetPosition()
{
	return NewtonSliderGetJointPosit( m_joint );
}

dFloat Slider::GetVelocity()
{
	return NewtonSliderGetJointVeloc( m_joint );
}

PyObject* Slider::GetForce()
{
	dFloat v[3];
	NewtonSliderGetJointForce( m_joint, v );
	PyObject* r = PyTuple_New( 3 );
	PyTuple_SetItem( r, 0, PyFloat_FromDouble( v[0] ) );
	PyTuple_SetItem( r, 1, PyFloat_FromDouble( v[1] ) );
	PyTuple_SetItem( r, 2, PyFloat_FromDouble( v[2] ) );
	return r;
}

dFloat Slider::CalculateStopAccel ( NewtonHingeSliderUpdateDesc* desc, dFloat position )
{
	return NewtonHingeCalculateStopAlpha ( m_joint, desc, position );
}

unsigned Slider::OnCallback( NewtonHingeSliderUpdateDesc* desc )
{
	if( !m_useLimits) return 0;
	dFloat distance;
	distance = NewtonSliderGetJointPosit (m_joint);
	if (distance < m_minLimit) {
		// if the distance is smaller than the predefine interval, stop the slider
		desc->m_accel = NewtonSliderCalculateStopAccel (m_joint, desc, m_minLimit);
		return 1;
	} else if (distance > m_maxLimit) {
		// if the distance is larger than the predefine interval, stop the slider
		desc->m_accel = NewtonSliderCalculateStopAccel (m_joint, desc, m_maxLimit);
		return 1;
	}

	// no action need it if the joint angle is with the limits
	return 0;

}


Corkscrew::Corkscrew( World* world, const dFloat* pivotPoint, const dFloat* pinDir, Body* childBody, Body* parentBody )
{
	m_world = world;
	m_joint = NewtonConstraintCreateCorkscrew( m_world->GetNewtonWorld(), pivotPoint, pinDir, childBody->m_body, parentBody->m_body );
	
}

dFloat Corkscrew::GetPosition( )
{
	return NewtonCorkscrewGetJointPosit( m_joint );
}

dFloat Corkscrew::GetAngle( )
{
	return NewtonCorkscrewGetJointAngle( m_joint );
}

dFloat Corkscrew::GetVelocity()
{
	return NewtonCorkscrewGetJointVeloc( m_joint );
}

dFloat Corkscrew::GetOmega ( )
{
	return NewtonCorkscrewGetJointOmega( m_joint );
}

PyObject* Corkscrew::GetForce()
{
	dFloat v[3];
	NewtonCorkscrewGetJointForce( m_joint, v );
	PyObject* r = PyTuple_New( 3 );
	PyTuple_SetItem( r, 0, PyFloat_FromDouble( v[0] ) );
	PyTuple_SetItem( r, 1, PyFloat_FromDouble( v[1] ) );
	PyTuple_SetItem( r, 2, PyFloat_FromDouble( v[2] ) );
	return r;
	
}

unsigned Corkscrew::OnCallback( NewtonHingeSliderUpdateDesc* desc )
{
	return 0;
}


dFloat Corkscrew::CalculateStopAlpha(  const NewtonHingeSliderUpdateDesc* desc, dFloat angle)
{
	return NewtonCorkscrewCalculateStopAlpha( m_joint, desc, angle );
}

dFloat Corkscrew::CalculateStopAccel (const NewtonJoint* corkscrew, const NewtonHingeSliderUpdateDesc* desc, dFloat position)
{
	return NewtonCorkscrewCalculateStopAccel( m_joint, desc, position );
}



UniversalJoint::UniversalJoint ( World* world, 
				const dFloat* pivotPoint, const dFloat* pinDir0, const dFloat* pinDir1, 
				Body* childBody, Body* parentBody)
{
	m_world = world;
	m_joint = NewtonConstraintCreateUniversal( m_world->GetNewtonWorld(), pivotPoint, pinDir0, pinDir1, childBody->m_body, parentBody->m_body );
}


dFloat UniversalJoint::GetAngle0 ()
{
	return NewtonUniversalGetJointAngle0( m_joint );
}

dFloat UniversalJoint::GetAngle1 ()
{
	return NewtonUniversalGetJointAngle1( m_joint );
}

dFloat UniversalJoint::GetOmega0 ()
{
	return NewtonUniversalGetJointOmega0( m_joint );
}

dFloat UniversalJoint::GetOmega1 ()
{
	return NewtonUniversalGetJointOmega1( m_joint );
}

PyObject* UniversalJoint::GetForce()
{
	dFloat v[3];
	NewtonUniversalGetJointForce( m_joint, v );
	PyObject* r = PyTuple_New( 3 );
	PyTuple_SetItem( r, 0, PyFloat_FromDouble( v[0] ) );
	PyTuple_SetItem( r, 1, PyFloat_FromDouble( v[1] ) );
	PyTuple_SetItem( r, 2, PyFloat_FromDouble( v[2] ) );
	return r;
}

dFloat UniversalJoint::CalculateStopAlpha0 ( const NewtonHingeSliderUpdateDesc* desc, dFloat angle )
{
	return NewtonUniversalCalculateStopAlpha0( m_joint, desc, angle );
}

dFloat UniversalJoint::CalculateStopAlpha1 ( const NewtonHingeSliderUpdateDesc* desc, dFloat angle )
{
	return NewtonUniversalCalculateStopAlpha1( m_joint, desc, angle );
}

unsigned UniversalJoint::OnCallback( NewtonHingeSliderUpdateDesc* desc )
{
	return 0;
}




UpVector::UpVector (World* world, const dFloat* pinDir, Body* body)
{
	m_world = world;
	m_joint = NewtonConstraintCreateUpVector( world->GetNewtonWorld(), pinDir, body->m_body );
}

PyObject* UpVector::GetPin ( )
{
	dFloat v[3];
	NewtonUpVectorGetPin( m_joint, v );
	PyObject* r = PyTuple_New( 3 );
	PyTuple_SetItem( r, 0, PyFloat_FromDouble( v[0] ) );
	PyTuple_SetItem( r, 1, PyFloat_FromDouble( v[1] ) );
	PyTuple_SetItem( r, 2, PyFloat_FromDouble( v[2] ) );
	return r;

}

void UpVector::SetPin ( const dFloat* pin )
{
	NewtonUpVectorSetPin( m_joint, pin );
}



Tire::Tire( Vehicle* v, void* tireid ) 
{
	m_vehicle  = v;
	m_tireID   = tireid;
	m_userData = NULL;
}

Tire::Tire( const Tire& t )
{
	m_vehicle = t.m_vehicle;
	m_tireID  = t.m_tireID;
	m_userData = t.m_userData;
	Py_XINCREF( m_userData );
}

Tire::~Tire ()
{
	Py_XDECREF( m_userData );
}

Tire& Tire::operator = ( Tire& t )
{
	Py_XDECREF( m_userData );
	m_vehicle = t.m_vehicle;
	m_tireID  = t.m_tireID;
	m_userData = t.m_userData;
	Py_XINCREF( m_userData );
	return *this;

}

void Tire::SetUserData( PyObject* userData)
{
	Py_XDECREF( m_userData );
	m_userData = userData;
	Py_XINCREF( m_userData );
}

PyObject* Tire::GetUserData( )
{
	return m_userData;
}

bool Tire::IsAirBorne()
{
	return ((NewtonVehicleTireIsAirBorne( m_vehicle->m_joint , m_tireID ) != 0 ) ? true : false);
}

bool Tire::LostSideGrip()
{
	return ((NewtonVehicleTireLostSideGrip( m_vehicle->m_joint, m_tireID ) != 0) ? true : false);
}

bool Tire::LostTraction()
{
	return ((NewtonVehicleTireLostTraction( m_vehicle->m_joint, m_tireID ) != 0 ) ? true : false);
}

dFloat Tire::GetOmega ()
{
	return NewtonVehicleGetTireOmega( m_vehicle->m_joint, m_tireID );
}

dFloat Tire::GetNormalLoad ()
{
	return NewtonVehicleGetTireNormalLoad( m_vehicle->m_joint, m_tireID );
}

dFloat Tire::GetSteerAngle ()
{
	return NewtonVehicleGetTireSteerAngle( m_vehicle->m_joint, m_tireID );
}

dFloat Tire::GetLateralSpeed ()
{
	return NewtonVehicleGetTireLateralSpeed( m_vehicle->m_joint, m_tireID );
}

dFloat Tire::GetLongitudinalSpeed ()
{
	return NewtonVehicleGetTireLongitudinalSpeed( m_vehicle->m_joint, m_tireID );
}

PyObject* Tire::GetMatrix ()
{
	dFloat matrix[16];
	 NewtonVehicleGetTireMatrix( m_vehicle->m_joint, m_tireID, matrix );
	PyObject* pylist = PyList_New( 16 );
	for(int i=0; i<16; i++ )
		PyList_SetItem( pylist, i, PyFloat_FromDouble( matrix[i] )  );
	return pylist;
}


void Tire::SetTorque ( dFloat torque)
{
	NewtonVehicleSetTireTorque( m_vehicle->m_joint, m_tireID, torque );
}

void Tire::SetSteerAngle ( dFloat angle)
{
	NewtonVehicleSetTireSteerAngle( m_vehicle->m_joint, m_tireID, angle );
}

void Tire::SetMaxSideSleepSpeed ( dFloat speed)
{
	NewtonVehicleSetTireMaxSideSleepSpeed( m_vehicle->m_joint, m_tireID, speed );
}

void Tire::SetSideSleepCoeficient (dFloat coeficient)
{
	NewtonVehicleSetTireSideSleepCoeficient( m_vehicle->m_joint, m_tireID, coeficient );
}

void Tire::SetMaxLongitudinalSlideSpeed (dFloat speed)
{
	NewtonVehicleSetTireMaxLongitudinalSlideSpeed( m_vehicle->m_joint, m_tireID, speed );
}

void Tire::SetLongitudinalSlideCoeficient (dFloat coeficient)
{
	NewtonVehicleSetTireLongitudinalSlideCoeficient( m_vehicle->m_joint, m_tireID, coeficient );
}

dFloat Tire::CalculateMaxBrakeAcceleration ()
{
	return NewtonVehicleTireCalculateMaxBrakeAcceleration( m_vehicle->m_joint, m_tireID );
}

void Tire::SetBrakeAcceleration ( dFloat accel, dFloat torqueLimit)
{
	return NewtonVehicleTireSetBrakeAcceleration( m_vehicle->m_joint, m_tireID, accel, torqueLimit);
}



Vehicle::Vehicle( World* world, const dFloat* upDir, Body* body)
{
	m_joint = NewtonConstraintCreateVehicle( world->GetNewtonWorld(), upDir, body->m_body );
	NewtonJointSetUserData( m_joint, this );

}

Vehicle::~Vehicle()
{
	NewtonDestroyJoint( m_world->GetNewtonWorld(), m_joint );
	while( tireList.size() > 0 )
		RemoveTire( tireList[0] );
}
	
void Vehicle::Reset()
{
	NewtonVehicleReset( m_joint );
}

Tire* Vehicle::AddTire( const dFloat* matrix, const dFloat* pin, dFloat mass, dFloat width, dFloat radius, 
	    dFloat suspensionShock, dFloat suspensionSpring, dFloat suspensionLength, PyObject* userData, int collisionID)
{
	void* id = NewtonVehicleAddTire( m_joint, matrix, pin, mass, width, radius, suspensionShock, suspensionSpring, suspensionLength, userData, collisionID );
	Tire* tire = new Tire ( this, id ) ;
	tireList.push_back( tire );
	
	return tire;
	
}

void Vehicle::RemoveTire( Tire* tire )
{
	NewtonVehicleRemoveTire( m_joint, tire->m_tireID );	
	std::vector<Tire*>::iterator it; 
	it = find( tireList.begin(), tireList.end(), tire );

	if ( it == tireList.end() )
		return;
	Tire* t = (*it)	;
	delete t;
	tireList.erase( it );
}


void Vehicle::OnCallback(  )
{

}

Tire* Vehicle::GetFirstTire( )
{
	void* id = NewtonVehicleGetFirstTireID( m_joint );
	
	//find tire with ID
	for (  unsigned int i=0; i<tireList.size(); i++ )
	{
		if( tireList[i]->m_tireID == id )
			return tireList[i];
	}

	return NULL;

}

Tire* Vehicle::GetNextTireID( Tire* tireID )
{
	void* id = NewtonVehicleGetNextTireID( m_joint, tireID->m_tireID );

	//find tire with ID
	for ( unsigned int i=0; i<tireList.size(); i++ )
	{
		if( tireList[i]->m_tireID == id)
			return tireList[i];
	}

	return NULL;
	//return Tire( this, id );
}

