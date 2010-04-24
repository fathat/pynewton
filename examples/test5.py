import sys
sys.path.append("../dist")

import newton
import time
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from framework.AppGL import *
from framework.listmatrix import *

#Callback function which causes the body to fall upwards


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
		collision2  = newton.Box( self.world, 2, 2, 2, None );
		self.body2 = newton.Body( self.world, collision2 ) 
		self.sliderJoint = newton.Slider( self.world, (-3, 0, 0), (1, 0, 0 ), self.body2, self.body)
		self.sliderJoint.SetMaxLimit( 2)
		del collision #release the collision
		del collision2 #release the collision
		
		self.body.SetMassMatrix( 1.0, 1.0, 1.0, 1.0 )
		self.body2.SetMassMatrix( 1.0, 1.0, 1.0, 1.0 )
		matrix =  [ 1, 0, 0, 0,
					0, 1, 0, 0,
					0, 0, 1, 0,
					-1, 0, 0, 1 ]
		matrix2 =  [ 1, 0, 0, 0,
					0, 1, 0, 0,
					0, 0, 1, 0,
					1, 0, 0, 1 ]
		self.body.SetMatrix( matrix )
		self.body2.SetMatrix( matrix2 )
		#self.body.SetOmega( (5.0, 5.0, 5.0 ) )
		m = self.body.GetMatrix( ) 
		PrintMatrix ( TransposeMatrix( m) )
		collision = self.body.GetCollision()
		print dir(collision)
		
		rval = collision.PointDistance( (0, 0, 0), m )
		print rval
		rval = collision.PointDistance( (-10, 0, 0), m)
		print rval
		del collision
		
		def A ( body ):
			mass, ix, iy, iz = self.body.GetMassMatrix()
			#self.body.AddForce( (0, -mass*0.5, 0) )
		def B ( body ):
			mass, ix, iy, iz = self.body2.GetMassMatrix()
			self.body2.AddForce( (0, -mass*2, 0) )
		self.body.ApplyForceAndTorqueCallback = A
		self.body2.ApplyForceAndTorqueCallback = B
		
	
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
		m2 = self.body2.GetMatrix()
		glPushMatrix()
		glTranslatef(0, 0, -10 )
		glPushMatrix()
		glMultMatrixf( m )
		DrawGLBox(1, 1, 1)
		glPopMatrix()
		
		glPushMatrix()
		glMultMatrixf( m2 )
		DrawGLBox(1, 1, 1)
		glPopMatrix()
		
		glPopMatrix()
		glutSwapBuffers()


if __name__ == '__main__':
	nt = NewtonTest(800, 600, 'secret opengl window')
	nt.MainLoop()