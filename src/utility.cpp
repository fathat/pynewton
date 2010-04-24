#include "utility.h"

PyObject* GetEulerAngle( const dFloat* matrix )
{
	dFloat eulerAngles[3];
	NewtonGetEulerAngle( matrix, eulerAngles );
	PyObject* pytuple = PyTuple_New( 3 );
	PyTuple_SetItem( pytuple, 0, PyFloat_FromDouble( eulerAngles[0] ) );
	PyTuple_SetItem( pytuple, 1, PyFloat_FromDouble( eulerAngles[1] ) );
	PyTuple_SetItem( pytuple, 2, PyFloat_FromDouble( eulerAngles[2] ) );

	return pytuple;
}

PyObject* SetEulerAngle( const dFloat* angles )
{
	dFloat matrix[16];
	NewtonSetEulerAngle( angles, matrix );
	PyObject* pylist = PyList_New( 16 );
	for(int i=0; i<16; i++ )
		PyList_SetItem( pylist, i, PyFloat_FromDouble( matrix[i] )  );
	return pylist;
}

