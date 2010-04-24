#Functions for creating/manipulating matrices in list form
def IdentMatrix():
	return [1, 0, 0, 0, 
			0, 1, 0, 0, 
			0, 0, 1, 0, 
			0, 0, 0, 1]

def TranslationMatrix( x, y, z ):
	return [1, 0, 0, 0, 
			0, 1, 0, 0, 
			0, 0, 1, 0, 
			x, y, z, 1]
			
def TransposeMatrix( m ):
	newmatrix = []
	for r in range(4):
		for c in range(4):
			newmatrix.append( m[r+c*4 ] )
	return newmatrix
						
def PrintMatrix( m ):
	for r in range(4):
		line = ""
		for c in range(4):
			line = line + str(m[c+r*4]) + " "
		print line
