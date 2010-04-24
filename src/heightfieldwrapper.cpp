#include "heightfieldwrapper.h"

HeightField::HeightField( World* world, int size, dFloat cellSize, PyObject* height_values ) {
	dFloat* hvals = new dFloat[size*size];
		
	for( int i=0; i<size*size; i++ ) {
		hvals[i] = (dFloat)PyFloat_AsDouble(PyList_GetItem(height_values, i));
	}

	hc = new HeightFieldCollision( world->GetNewtonWorld(), size, cellSize, hvals );

	delete [] hvals;
}

HeightField::~HeightField()
{
	delete hc;
}

void HeightField::SetMatrix ( const dFloat* matrix )
{
	hc->SetMatrix(matrix);
}

PyObject* HeightField::GetMatrix()
{
	PyObject* obj = PyList_New(0);
	dFloat* matrix = hc->GetMatrix();

	for( int i=0; i<16; i++) {
		PyList_Append( obj,  PyFloat_FromDouble( matrix[i]));
	}
	return obj;

}

dFloat HeightField::CellSize() 
{ 
	return hc->CellSize(); 
}

int    HeightField::Size() 
{ 
	return hc->Size(); 
}

