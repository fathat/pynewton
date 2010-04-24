#ifndef UTILITY_H
#define UTILITY_H

#include <Python.h>
#include <Newton.h>

PyObject* GetEulerAngle( const dFloat* matrix );
PyObject* SetEulerAngle( const dFloat* angle );

#endif 

