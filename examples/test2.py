import sys
sys.path.append("../dist")

import time
import newton
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from framework.AppGL import *
from framework.listmatrix import *


#Callback for applying force and torque to a body (in this case, it
#applies gravity)
def ApplyForceAndTorque(body):
	mass, ix, iy, iz = body.GetMassMatrix()
	body.SetForce( (0, -mass*9.8, 0) )


#Simple class for making a drawable box that falls
class FallingBox(object):
	
	def __init__(self, world, startingPosition):
		self.world = world
		self.collision = newton.Box( self.world, 2, 2, 2, None );
		self.body = newton.Body( self.world, self.collision )
		self.body.ApplyForceAndTorqueCallback = ApplyForceAndTorque
		del self.collision
		
		self.body.SetMassMatrix( 1.0, 1.0, 1.0, 1.0 )
		
		x, y, z = startingPosition
		matrix = TranslationMatrix( x, y, z )
		self.body.SetMatrix( matrix )
		
	
	def Draw( self ):
		m = self.body.GetMatrix( )
		glPushMatrix()
		glMultMatrixf( m )
		DrawGLBox(1, 1, 1)
		glPopMatrix()


class NewtonTest (AppGL):
	
	def OnInit( self ):
		self.sphere = gluNewQuadric()
		self.InitPhysics()
		self.timeDelta = 0
		self.lastTime = time.clock()
		texture = LoadTexture("crate.bmp")
		
	def InitPhysics( self ):
		self.world = newton.World()
		
		self.boxes = []
		for x in range( 5 ):
			for z in range ( 5 ):
				for y in range ( 5 ):
					self.boxes.append( FallingBox( self.world, ( x*4, y*4, z*4 ) ) )
		
		print "Making floor box"
		floorBox = newton.Box( self.world, 200, 4, 200, None )
		self.floorBody = newton.Body( self.world, floorBox )
		del floorBox
		
		self.floorBody.SetMatrix( TranslationMatrix( 0, -5, 0 ) )
		print "Done initializing physics" 
		
	def Update ( self ):
		t = time.clock()
		timeDelta = t - self.lastTime
		self.lastTime = t
		self.world.Update(timeDelta)
		self.DrawScene()
		
		
	
	def DrawScene ( self ):
		glEnable( GL_TEXTURE_2D)
		glClearColor( 0.5, 0.5, 0.5, 1 )
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
		glLoadIdentity()
		
		glPushMatrix()
		glTranslatef(-8, -1, -40 )
		for b in self.boxes:
			b.Draw()
		
		glTranslatef(0, -5, 0 )
		DrawGLBox( 100, 2, 100 )
		glPopMatrix()
		glutSwapBuffers()
	


if __name__ == '__main__':
	nt = NewtonTest(1024, 768, 'secret opengl window')
	nt.MainLoop()