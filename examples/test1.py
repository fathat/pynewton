import sys
sys.path.append("../dist")

import time
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from framework.AppGL import *
from framework.listmatrix import *
import newton

class NewtonTest (AppGL):
	
	def OnInit( self ):
		self.sphere = gluNewQuadric()
		self.InitPhysics()
		self.lastTime = time.clock()
		texture = LoadTexture("crate.bmp")
	
	def InitPhysics( self ):
		#Create the world, box and body
		self.world = newton.World()
		collision  = newton.Box( self.world, 2, 2, 2, None );
		self.body  = newton.Body( self.world, collision ) 
		del collision #release the collision
		
		self.body.SetMassMatrix( 1.0, 1.0, 1.0, 1.0 )
		matrix =  [ 1, 0, 0, 0,
					0, 1, 0, 0,
					0, 0, 1, 0,
					0, 0, 0, 1 ]
		self.body.SetMatrix( matrix )
		self.body.SetOmega( (5.0, 5.0, 5.0 ) )
		m = self.body.GetMatrix( ) 
		PrintMatrix ( TransposeMatrix( m) )
		collision = self.body.GetCollision()
		print dir(collision)
		
		rval = collision.PointDistance( (0, 0, 0), m )
		print rval
		rval = collision.PointDistance( (-10, 0, 0), m)
		print rval
		del collision
		
	
	def Update ( self ):
		t = time.clock()
		timeDelta = t - self.lastTime
		self.lastTime = t
		self.world.Update(timeDelta)
		self.DrawScene()
	
	def DrawScene ( self ):
		glEnable( GL_TEXTURE_2D)
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
		glLoadIdentity()
		m = self.body.GetMatrix()
		glPushMatrix()
		glTranslatef(0, 0, -10 )
		glMultMatrixf( m )
		DrawGLBox(1, 1, 1)
		glPopMatrix()
		glutSwapBuffers()


if __name__ == '__main__':
	nt = NewtonTest(800, 600, 'secret opengl window')
	nt.MainLoop()