%module(directors="1") pynewton
%{
#include <Python.h>
#include "wrapper.h"
#include <Newton.h>

//Have to undef these two symbols for g++ 4.0 to compile the code
#undef min
#undef max
%}

%include "carrays.i"
%array_functions( double, CDoubleArray);

typedef float dFloat;

%typemap(in) const dFloat* matrix {
	
	/* Check if is a list */
	if (PyList_Check($input)) 
	{
		int size = PyList_Size($input);
		int i = 0;
		$1 = (dFloat*) malloc(size*sizeof(dFloat));
		for (i = 0; i < size; i++) 
		{
			PyObject *o = PyList_GetItem($input,i);
			$1[i] = (dFloat)PyFloat_AsDouble( o );
		}
		
	} 
	else if ( $input == Py_None )
	{
		$1 = NULL;
	}
	
	else 
	{
		PyErr_SetString(PyExc_TypeError,"not a list");
		return NULL;
	}
}

// This cleans up the char ** array we malloc'd before the function call
%typemap(freearg) const dFloat* matrix {
	free((dFloat *) $1);
}


%typemap(in) const dFloat* vector {
	
	/* Check if is a list */
	if (PyTuple_Check($input)) 
	{
		int size = PyTuple_Size($input);
		int i = 0;
		$1 = (dFloat*) malloc(size*sizeof(dFloat));
		for (i = 0; i < size; i++) 
		{
			PyObject *o = PyTuple_GetItem($input,i);
			$1[i] = (dFloat)PyFloat_AsDouble( o );
		}
		
	} 
	else if ( $input == Py_None )
	{
		$1 = NULL;
	}
	
	else 
	{
		PyErr_SetString(PyExc_TypeError,"not a tuple");
		return NULL;
	}
}

// This cleans up the char ** array we malloc'd before the function call
%typemap(freearg) const dFloat* vector {
	free((dFloat *) $1);
}


//wrapper functions
%apply const dFloat* matrix { const dFloat* offsetMatrix };

%apply const dFloat* vector { const dFloat* angle };
%apply const dFloat* vector { const dFloat* plane };
%apply const dFloat* vector { const dFloat* omega };
%apply const dFloat* vector { const dFloat* force };
%apply const dFloat* vector { const dFloat* torque };
%apply const dFloat* vector { const dFloat* velocity };
%apply const dFloat* vector { const dFloat* dir };
%apply const dFloat* vector { const dFloat* dir0 };
%apply const dFloat* vector { const dFloat* dir1 };
%apply const dFloat* vector { const dFloat* directionVector };
%apply const dFloat* vector { const dFloat* gravityVector };
%apply const dFloat* vector { const dFloat* vectorA };
%apply const dFloat* vector { const dFloat* vectorB };
%apply const dFloat* vector { const dFloat* point };
%apply const dFloat* vector { const dFloat* pin };
%apply const dFloat* vector { const dFloat* pinDir };
%apply const dFloat* vector { const dFloat* p0 };
%apply const dFloat* vector { const dFloat* p1 };
%apply const dFloat* vector { const dFloat* pinDir0 };
%apply const dFloat* vector { const dFloat* pinDir1 };
%apply const dFloat* vector { const dFloat* pivotPoint };
%apply const dFloat* vector { const dFloat* minPoint };
%apply const dFloat* vector { const dFloat* maxPoint };

%feature("director") Body;
%feature("director") MaterialGroups;
%feature("director") World;
%feature("director") NewtonContactBeginCallbackProxy;
%feature("director") NewtonContactProcessCallbackProxy;
%feature("director") NewtonContactEndCallbackProxy;
%feature("director") BallJoint;
%feature("director") Hinge;
%feature("director") Slider;
%feature("director") Corkscrew;
%feature("director") UniversalJoint;
%feature("director") UpVector;
%feature("director") Tire;
%feature("director") Vehicle;
%feature("director") TreeCollisionUserCallback;

%feature("autodoc", "1");
%feature("docstring");
%include "materialgroup.h"
%include "world.h"
%include "material.h"
%include "collision.h"
%include "body.h"
%include "joint.h"
%include "utility.h"
%include "heightfieldwrapper.h"


