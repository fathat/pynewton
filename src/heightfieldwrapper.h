#pragma once

#include <Python.h>
#include "HeightFieldCollision.h"
#include "world.h"

class HeightField {
protected:
	HeightFieldCollision* hc;

public:
	HeightField( World* world, int size, dFloat cellSize, PyObject* height_values );
	virtual ~HeightField();

	void SetMatrix ( const dFloat* matrix );
	PyObject* GetMatrix();

	dFloat CellSize() ;
	int    Size() ;

};

