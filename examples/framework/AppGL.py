import sys
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import Image




class AppGL(object):
	def __init__(self, width, height, title ):
		glutInit( sys.argv )
		glutInitDisplayMode( GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH)
		glutInitWindowSize( width, height )
		glutInitWindowPosition( 30, 60 )
		self.window = glutCreateWindow( title )

		glutDisplayFunc( self.DrawScene )
		glutIdleFunc( self.Update )
		glutReshapeFunc( self.Resize )
		glutKeyboardFunc( self.KeyPressed ) 
		self.Width	= width
		self.Height = height
		self.InitRenderStates()
		self.OnInit()
		
	def OnInit( self ):
		pass
	
	def InitRenderStates ( self ):
		#initialize basic states
		glClearColor(0.0, 0.0, 0.0, 0.0)
		glClearDepth(1.0)
		glDepthFunc(GL_LESS)
		glEnable(GL_DEPTH_TEST)
		glShadeModel(GL_SMOOTH)

		#set the projection matrix
		glMatrixMode(GL_PROJECTION)
		glLoadIdentity()					
		gluPerspective(45.0, float(self.Width)/float(self.Height), 0.1, 100.0)
		#return to modelview
		glMatrixMode(GL_MODELVIEW)
		
	def Resize( self, width, height ):
		self.Width	= width
		self.Height = height
		self.OnResize( width, height )
		
	def OnResize( self, width, height ):
		pass

	def Update( self ):
		self.DrawScene()
		pass

	def OnResize( self, width, height ):
		if height == 0: 					# Prevent A Divide By Zero If The Window Is Too Small 
			height = 1

		glViewport(0, 0, width, height) 	# Reset The Current Viewport And Perspective Transformation
		glMatrixMode(GL_PROJECTION)
		glLoadIdentity()
		gluPerspective(45.0, float(width)/float(height), 0.1, 100.0)
		glMatrixMode(GL_MODELVIEW)


	def DrawScene(self ):
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
		glutSwapBuffers()

	# The function called whenever a key is pressed. Note the use of Python tuples to pass in: (key, x, y)	
	def KeyPressed(self, *args):
		# If escape is pressed, kill everything.
		if args[0] == '\x1B':
			glutDestroyWindow(self.window)
			sys.exit()

	def MainLoop(self ):
		glutMainLoop()
		

def LoadTexture( filename ):
    #global texture
    image = Image.open(filename)
	
    ix = image.size[0]
    iy = image.size[1]
    image = image.tostring("raw", "RGBX", 0, -1)
	
    # Create Texture	
    # There does not seem to be support for this call or the version of PyOGL I have is broken.
    
    texture = glGenTextures(1)
    glBindTexture(GL_TEXTURE_2D, texture)   # 2d texture (x and y size)
	
    glPixelStorei(GL_UNPACK_ALIGNMENT,1)
    glTexImage2D(GL_TEXTURE_2D, 0, 3, ix, iy, 0, GL_RGBA, GL_UNSIGNED_BYTE, image)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL)
    return texture
		
def DrawGLBox(w, h, d):
	glBegin(GL_QUADS )
	# Front Face (note that the texture's corners have to match the quad's corners)
	glTexCoord2f(0.0, 0.0); glVertex3f(-w, -h,  d)	# Bottom Left Of The Texture and Quad
	glTexCoord2f(1.0, 0.0); glVertex3f( w, -h,  d)	# Bottom Right Of The Texture and Quad
	glTexCoord2f(1.0, 1.0); glVertex3f( w,  h,  d)	# Top Right Of The Texture and Quad
	glTexCoord2f(0.0, 1.0); glVertex3f(-w,  h,  d)	# Top Left Of The Texture and Quad
	
	# Back Face
	glTexCoord2f(1.0, 0.0); glVertex3f(-w, -h, -d)	# Bottom Right Of The Texture and Quad
	glTexCoord2f(1.0, 1.0); glVertex3f(-w,  h, -d)	# Top Right Of The Texture and Quad
	glTexCoord2f(0.0, 1.0); glVertex3f( w,  h, -d)	# Top Left Of The Texture and Quad
	glTexCoord2f(0.0, 0.0); glVertex3f( w, -h, -d)	# Bottom Left Of The Texture and Quad
	
	# Top Face
	glTexCoord2f(0.0, 1.0); glVertex3f(-w,  h, -d)	# Top Left Of The Texture and Quad
	glTexCoord2f(0.0, 0.0); glVertex3f(-w,  h,  d)	# Bottom Left Of The Texture and Quad
	glTexCoord2f(1.0, 0.0); glVertex3f( w,  h,  d)	# Bottom Right Of The Texture and Quad
	glTexCoord2f(1.0, 1.0); glVertex3f( w,  h, -d)	# Top Right Of The Texture and Quad
	
	# Bottom Face       
	glTexCoord2f(1.0, 1.0); glVertex3f(-w, -h, -d)	# Top Right Of The Texture and Quad
	glTexCoord2f(0.0, 1.0); glVertex3f( w, -h, -d)	# Top Left Of The Texture and Quad
	glTexCoord2f(0.0, 0.0); glVertex3f( w, -h,  d)	# Bottom Left Of The Texture and Quad
	glTexCoord2f(1.0, 0.0); glVertex3f(-w, -h,  d)	# Bottom Right Of The Texture and Quad
	
	# Right face
	glTexCoord2f(1.0, 0.0); glVertex3f( w, -h, -d)	# Bottom Right Of The Texture and Quad
	glTexCoord2f(1.0, 1.0); glVertex3f( w,  h, -d)	# Top Right Of The Texture and Quad
	glTexCoord2f(0.0, 1.0); glVertex3f( w,  h,  d)	# Top Left Of The Texture and Quad
	glTexCoord2f(0.0, 0.0); glVertex3f( w, -h,  d)	# Bottom Left Of The Texture and Quad
	
	# Left Face
	glTexCoord2f(0.0, 0.0); glVertex3f(-w, -h, -d)	# Bottom Left Of The Texture and Quad
	glTexCoord2f(1.0, 0.0); glVertex3f(-w, -h,  d)	# Bottom Right Of The Texture and Quad
	glTexCoord2f(1.0, 1.0); glVertex3f(-w,  h,  d)	# Top Right Of The Texture and Quad
	glTexCoord2f(0.0, 1.0); glVertex3f(-w,  h, -d)	# Top Left Of The Texture and Quad
	glEnd() 						# Done Drawing The Quad

		
if __name__ == '__main__':
	app = AppGL(800, 600, "Secret Window" )
	app.MainLoop()

