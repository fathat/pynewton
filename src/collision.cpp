#include <Newton.h>
#include "collision.h"
#include "body.h"


void SerializeFile (void* serializeHandle, const void* buffer, size_t size)
{
	fwrite (buffer, size, 1, (FILE*) serializeHandle);
}

void DeSerializeFile (void* serializeHandle, void* buffer, size_t size)
{
	fread (buffer, size, 1, (FILE*) serializeHandle);
}


void GeneralNewtonTreeCollisionCallback (const NewtonBody* bodyWithTreeCollision, const NewtonBody* body,
										 const dFloat* vertex, int vertexstrideInBytes, 
										 int indexCount, const int* indexArray)
{
	Body* wrapperA = (Body*)NewtonBodyGetUserData( bodyWithTreeCollision );
	Body* wrapperB = (Body*)NewtonBodyGetUserData( body );

	wrapperA->OnTreeCollisionWith( wrapperB );

}


CollisionGeometry::CollisionGeometry()
{
	m_collision = NULL;
	m_world = NULL;
	
}

CollisionGeometry::~CollisionGeometry() 
{
	NewtonReleaseCollision( m_world, this->m_collision );
}



NewtonCollision* CollisionGeometry::GetNewtonCollision() { return m_collision; }
NewtonWorld* CollisionGeometry::GetNewtonWorld() { return m_world; }

PyObject* CollisionGeometry::PointDistance ( const dFloat *point, const dFloat* matrix)
{
	dFloat contact_array[3];
	dFloat normal_array[3];

	int rval = NewtonCollisionPointDistance ( m_world, point, m_collision, matrix, contact_array, normal_array );

	if( rval == 1 )
	{
		PyObject* contact_and_normal = PyTuple_New( 6 ) ;
		
		for ( int i=0; i<3; i++ ) {
			PyTuple_SetItem( contact_and_normal, i,    PyFloat_FromDouble( contact_array[i] ) );
			PyTuple_SetItem( contact_and_normal, i+3,  PyFloat_FromDouble( normal_array[i]  ) );
		}
		return contact_and_normal;
	}

	return Py_None;
}


PyObject* CollisionGeometry::ClosestPoint ( const dFloat* matrixA, CollisionGeometry* collisionB, const dFloat* matrixB )
{
/*	int NewtonCollisionClosestPoint(
const NewtonWorld* newtonWorld,
const NewtonCollision* collisionA,
const dFloat* matrixA,
const NewtonCollision* collisionB,
const dFloat* matrixB,
dFloat* contactA,
dFloat* contactB,
dFloat* normalAB) */

	dFloat contactA[3];
	dFloat contactB[3];
	dFloat normalAB[3];

	int rval = NewtonCollisionClosestPoint( GetNewtonWorld(), m_collision, matrixA, 
								 collisionB->GetNewtonCollision(), matrixB, 
								 contactA, contactB, normalAB );

	if( rval )
	{
		PyObject* t = PyTuple_New( 9 ) ;
		
		for ( int i=0; i<3; i++ ) {
			PyTuple_SetItem( t, i,    PyFloat_FromDouble( contactA[i] ) );
			PyTuple_SetItem( t, i+3,  PyFloat_FromDouble( contactB[i]  ) );
			PyTuple_SetItem( t, i+6,  PyFloat_FromDouble( normalAB[i]  ) );
		}
		return t;
	}

	return Py_None;

}

PyObject* CollisionGeometry::CalculateAABB( const dFloat* matrix )
{
	dFloat v1[3];
	dFloat v2[3];

	NewtonCollisionCalculateAABB(m_collision,matrix, v1, v2) ;

	PyObject* r = PyTuple_New( 6 );
	PyTuple_SetItem( r, 0, PyFloat_FromDouble( v1[0] ) );
	PyTuple_SetItem( r, 1, PyFloat_FromDouble( v1[1] ) );
	PyTuple_SetItem( r, 2, PyFloat_FromDouble( v1[2] ) );
	PyTuple_SetItem( r, 3, PyFloat_FromDouble( v2[0] ) );
	PyTuple_SetItem( r, 4, PyFloat_FromDouble( v2[1] ) );
	PyTuple_SetItem( r, 5, PyFloat_FromDouble( v2[2] ) );

	return r;
	
}

RayCollision CollisionGeometry::RayCast ( const dFloat* vectorA, const dFloat* vectorB ) 
{
	dFloat n[3];
	RayCollision returnVal;
	returnVal.distance = NewtonCollisionRayCast(
		m_collision,
		vectorA,
		vectorB,
		n,
		&returnVal.attributeID) ;
	returnVal.normalX = n[0];
	returnVal.normalY = n[1];
	returnVal.normalZ = n[2];
	return returnVal;
}

CompoundCollision::CompoundCollision( World* world )
{
	m_world = world->GetNewtonWorld();
}

void CompoundCollision::AddCollision( CollisionGeometry* c )
{
	m_geoms.push_back( c->GetNewtonCollision() ) ;
}

void CompoundCollision::Finalize()
{
	//put vector into a flat array
	NewtonCollision** cc = new NewtonCollision*[m_geoms.size()];
	for( unsigned int i=0; i<m_geoms.size(); i++)
		cc[i] = m_geoms.at( i );

	m_collision = NewtonCreateCompoundCollision( m_world, (int)m_geoms.size(), cc );
	delete[] cc;
}


Sphere::Sphere( World* world, dFloat radiusX, dFloat radiusY, dFloat radiusZ, const dFloat* offsetMatrix )
{
	m_world = world->GetNewtonWorld();
	m_collision = NewtonCreateSphere( m_world, radiusX, radiusY, radiusZ, offsetMatrix );
}

Box::Box(World* world, dFloat x, dFloat y, dFloat z, const dFloat* offsetMatrix )
{
	m_world = world->GetNewtonWorld();
	m_collision = NewtonCreateBox( m_world, x, y, z, offsetMatrix );
	
}


Cone::Cone( World* world, dFloat radius, dFloat height, const dFloat* offsetMatrix )
{
	m_world = world->GetNewtonWorld();
	m_collision = NewtonCreateCone( m_world, radius, height, offsetMatrix );
}

Capsule::Capsule( World* world, dFloat radius, dFloat height, const dFloat* offsetMatrix )
{
	m_world = world->GetNewtonWorld();
	m_collision = NewtonCreateCapsule( m_world, radius, height, offsetMatrix );
}

Cylinder::Cylinder( World* world, dFloat radius, dFloat height, const dFloat* offsetMatrix )
{
	m_world = world->GetNewtonWorld();
	m_collision = NewtonCreateCylinder( m_world, radius, height, offsetMatrix );
}

ChamferCylinder::ChamferCylinder( World* world, dFloat radius, dFloat height, const dFloat* offsetMatrix )
{
	m_world = world->GetNewtonWorld();
	m_collision = NewtonCreateChamferCylinder ( m_world, radius, height, offsetMatrix );

}

ConvexHull::ConvexHull ( World* world, PyObject* pylist, const dFloat* offsetMatrix )
{
	m_world = world->GetNewtonWorld();
	
	int count = PyList_Size( pylist );
	dFloat* vertexCloud = new dFloat[ count ] ;


	m_collision = NewtonCreateConvexHull( m_world, count/3, vertexCloud, sizeof(dFloat)*3, (dFloat*)offsetMatrix );
}

void ConvexHull::SetUserID( unsigned id )
{
	NewtonConvexCollisionSetUserID( m_collision, id );
}

unsigned int ConvexHull::GetUserID()
{
	return NewtonConvexCollisionGetUserID( m_collision );
}

dFloat ConvexHull::CalculateVolume( )
{
	return NewtonConvexCollisionCalculateVolume( m_collision );
}

PyObject* ConvexHull::CalculateInertiaMatrix ( )
{
	dFloat inertia[3];
	dFloat origin[3];
	NewtonConvexCollisionCalculateInertialMatrix( m_collision, inertia, origin );

	PyObject* r = PyTuple_New( 6 );
	PyTuple_SetItem( r, 0, PyFloat_FromDouble( inertia[0] ) );
	PyTuple_SetItem( r, 1, PyFloat_FromDouble( inertia[1] ) );
	PyTuple_SetItem( r, 2, PyFloat_FromDouble( inertia[2] ) );
	PyTuple_SetItem( r, 3, PyFloat_FromDouble( origin[0] ) );
	PyTuple_SetItem( r, 4, PyFloat_FromDouble( origin[1] ) );
	PyTuple_SetItem( r, 5, PyFloat_FromDouble( origin[2] ) );

	return r;
}

ConvexHullModifier::ConvexHullModifier( World* world, ConvexHull* hull )
{
	m_world = world->GetNewtonWorld();
	m_hull = hull;
	
	m_collision = NewtonCreateConvexHullModifier( m_world, hull->GetNewtonCollision() );
}

void ConvexHullModifier::SetMatrix( const dFloat* matrix )
{
	NewtonConvexHullModifierSetMatrix( m_hull->GetNewtonCollision(), matrix );
}

PyObject* ConvexHullModifier::GetMatrix()
{
	dFloat matrix[16];
	NewtonConvexHullModifierGetMatrix( m_hull->GetNewtonCollision(), matrix );
	PyObject* pylist = PyList_New( 16 );
	for(int i=0; i<16; i++ )
		PyList_SetItem( pylist, i, PyFloat_FromDouble( matrix[i] )  );
	return pylist;
}


void TreeCollisionUserCallback::OnCallback  ( Body* bodyWithTreeCollision, Body* body,
						  const dFloat* vertex, int vertexstrideInBytes, 
						  int indexCount, const int* indexArray)
{
}

TreeCollision::TreeCollision( World* world ) 
{
	m_collision = NewtonCreateTreeCollision( world->GetNewtonWorld(), GeneralNewtonTreeCollisionCallback );
}

void TreeCollision::BeginBuild()
{
	NewtonTreeCollisionBeginBuild( m_collision );
}

void TreeCollision::AddFace( PyObject* vertexList, int faceAttribute ) 
{
	int vertexCount   = PyList_Size( vertexList ) /3 ;
	int strideInBytes = sizeof( dFloat) * 3;

	dFloat* v = new dFloat[vertexCount*3];
	for( int i=0; i<vertexCount*3; i++)
		v[i] = (dFloat)PyFloat_AsDouble( PyList_GetItem( vertexList, i ) );


	NewtonTreeCollisionAddFace( m_collision, vertexCount, v, strideInBytes, faceAttribute );
	delete[] v;
}

void TreeCollision::EndBuild( int optimize)
{
	NewtonTreeCollisionEndBuild( m_collision, optimize);
}

int  TreeCollision::GetFaceAttribute ( const int* faceIndexArray ) 
{
	return NewtonTreeCollisionGetFaceAtribute( m_collision, faceIndexArray );
}

void TreeCollision::SetFaceAttribute( const int* faceIndexArray, int attribute ) 
{
	NewtonTreeCollisionSetFaceAtribute( m_collision, faceIndexArray, attribute );
}

bool TreeCollision::Serialize( const char* filename )
{
	// save the collision file
	FILE* file = fopen (filename, "wb");
	if(!file) return false;
	NewtonTreeCollisionSerialize (this->m_collision, SerializeFile, file);
	fclose (file);
	return true;
}

bool TreeCollision::Deserialize( const char* filename )
{
	NewtonReleaseCollision( m_world, this->m_collision );

	FILE* file = fopen (filename, "rb");
	if( file == NULL ) return false;
	m_collision = NewtonCreateTreeCollisionFromSerialization (this->m_world, GeneralNewtonTreeCollisionCallback, DeSerializeFile, file);
	fclose (file);
	return true;
}


NullCollider::NullCollider( World* world )
{
	m_world = world->GetNewtonWorld();
	m_collision = NewtonCreateNull( m_world );
}
