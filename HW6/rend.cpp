/* CS580 Homework 3 */

#include	"stdafx.h"
#include	"stdio.h"
#include	"math.h"
#include	"Gz.h"
#include	"rend.h"
#include <algorithm>				/* std::sort() */

#define PI (float) 3.14159265358979323846

int GzRender::GzRotXMat(float degree, GzMatrix mat)
{
/* HW 3.1
// Create rotate matrix : rotate along x axis
// Pass back the matrix using mat value
*/
	float theta = dtor(degree);
	resetMatrix(mat);

	mat[0][0] = 1.0f;
	mat[1][1] = (float)cos(theta);
	mat[1][2] = (float)-sin(theta);
	mat[2][1] = (float)sin(theta);
	mat[2][2] = (float)cos(theta);
	mat[3][3] = 1.0f;
	return GZ_SUCCESS;
}

int GzRender::GzRotYMat(float degree, GzMatrix mat)
{
/* HW 3.2
// Create rotate matrix : rotate along y axis
// Pass back the matrix using mat value
*/
	float theta = dtor(degree);
	resetMatrix(mat);

	mat[0][0] = (float)cos(theta);
	mat[0][2] = (float)sin(theta);
	mat[1][1] = 1.0f;
	mat[2][0] = (float)-sin(theta);
	mat[2][2] = (float)cos(theta);
	mat[3][3] = 1.0f;
	return GZ_SUCCESS;
}

int GzRender::GzRotZMat(float degree, GzMatrix mat)
{
/* HW 3.3
// Create rotate matrix : rotate along z axis
// Pass back the matrix using mat value
*/
	float theta = dtor(degree);
	resetMatrix(mat);

	mat[0][0] = (float)cos(theta);
	mat[0][1] = (float)-sin(theta);
	mat[1][0] = (float)sin(theta);
	mat[1][1] = (float)cos(theta);
	mat[2][2] = 1.0f;
	mat[3][3] = 1.0f;
	return GZ_SUCCESS;
}

int GzRender::GzTrxMat(GzCoord translate, GzMatrix mat)
{
/* HW 3.4
// Create translation matrix
// Pass back the matrix using mat value
*/
	resetMatrix(mat);
	/* Set diagonals to 1.0f */
	for (int i = 0; i < 4; i++) {
		mat[i][i] = 1.0f;
	}

	mat[0][3] = translate[0];
	mat[1][3] = translate[1];
	mat[2][3] = translate[2];
	return GZ_SUCCESS;
}


int GzRender::GzScaleMat(GzCoord scale, GzMatrix mat)
{
/* HW 3.5
// Create scaling matrix
// Pass back the matrix using mat value
*/
	resetMatrix(mat);
	for (int i = 0; i < 3; i++) {
		mat[i][i] = scale[i];
	}
	mat[3][3] = 1.0f;
	return GZ_SUCCESS;
}


GzRender::GzRender(int xRes, int yRes)
{
/* HW1.1 create a framebuffer for MS Windows display:
 -- set display resolution
 -- allocate memory for framebuffer : 3 bytes(b, g, r) x width x height
 -- allocate memory for pixel buffer
 */
	xres = xRes;
	yres = yRes;
	framebuffer = new char[3 * xRes * yRes];
	pixelbuffer = new GzPixel[xRes * yRes];

/* HW 3.6
- setup Xsp and anything only done once 
- init default camera 
*/ 
	matlevel = -1;	/* Stack counter */
	numlights = 0;	/* number of lights */

	/* Setup Xsp matrix */
	resetMatrix(Xsp);
	Xsp[0][0] = (float)xRes / 2.0f;
	Xsp[0][3] = (float)xRes / 2.0f;
	Xsp[1][1] = (float)-yRes / 2.0f;
	Xsp[1][3] = (float)yRes / 2.0f;
	Xsp[2][2] = (float)MAXINT;
	Xsp[3][3] = 1.0f;

	/* Initialize default camera */
	m_camera.position[X] = DEFAULT_IM_X;
	m_camera.position[Y] = DEFAULT_IM_Y;
	m_camera.position[Z] = DEFAULT_IM_Z;
	for (int i = 0; i < 3; i++) {
		m_camera.lookat[i] = 0.0f;
		m_camera.worldup[i] = 0.0f;
	}
	m_camera.worldup[Y] = 1.0f; /* up vector is y-axis */
	m_camera.FOV = DEFAULT_FOV;
}

GzRender::~GzRender()
{
/* HW1.2 clean up, free buffer memory */
	delete[] framebuffer;
	delete[] pixelbuffer;
}

int GzRender::GzDefault()
{
/* HW1.3 set pixel buffer to some default values - start a new frame */
	int res = xres * yres;
	for (int i = 0; i < res; i++) {
		pixelbuffer[i] = { C_LIMIT, 2000, 0, 1, MAXINT };
	}
	return GZ_SUCCESS;
}

int GzRender::GzBeginRender()
{
/* HW 3.7 
- setup for start of each frame - init frame buffer color,alpha,z
- compute Xiw and projection xform Xpi from camera definition 
- init Ximage - put Xsp at base of stack, push on Xpi and Xiw 
- now stack contains Xsw and app can push model Xforms when needed 
*/ 
	GzCoord cl, up, upprime, Xvector, Yvector, Zvector;
	float Mcl, upDotZ, Mup;

	for (int i = 0; i < 3; i++) {
		up[i] = m_camera.worldup[i];
	}

	for (int i = 0; i < 3; i++) {
		cl[i] = m_camera.lookat[i] - m_camera.position[i];
	}

	Mcl = (float)sqrt(cl[0] * cl[0] + cl[1] * cl[1] + cl[2] * cl[2]);

	for (int i = 0; i < 3; i++) {
		Zvector[i] = cl[i] / Mcl;
	}

	upDotZ = m_camera.worldup[0] * Zvector[0] + m_camera.worldup[1] * Zvector[1] + m_camera.worldup[2] * Zvector[2];
	for (int i = 0; i < 3; i++) {
		upprime[i] = up[i] - upDotZ * Zvector[i];
	}

	Mup = (float)sqrt(upprime[0] * upprime[0] + upprime[1] * upprime[1] + upprime[2] * upprime[2]);

	for (int i = 0; i < 3; i++) {
		Yvector[i] = upprime[i] / Mup;
	}

	Xvector[0] = Yvector[Y] * Zvector[Z] - Yvector[Z] * Zvector[Y];
	Xvector[1] = Yvector[Z] * Zvector[X] - Yvector[X] * Zvector[Z];
	Xvector[2] = Yvector[X] * Zvector[Y] - Yvector[Y] * Zvector[X];

	m_camera.Xiw[0][0] = Xvector[0];
	m_camera.Xiw[0][1] = Xvector[1];
	m_camera.Xiw[0][2] = Xvector[2];
	m_camera.Xiw[0][3] = -(Xvector[X] * m_camera.position[X] + Xvector[Y] * m_camera.position[Y] + Xvector[Z] * m_camera.position[Z]);

	m_camera.Xiw[1][0] = Yvector[0];
	m_camera.Xiw[1][1] = Yvector[1];
	m_camera.Xiw[1][2] = Yvector[2];
	m_camera.Xiw[1][3] = -(Yvector[X] * m_camera.position[X] + Yvector[Y] * m_camera.position[Y] + Yvector[Z] * m_camera.position[Z]);

	m_camera.Xiw[2][0] = Zvector[0];
	m_camera.Xiw[2][1] = Zvector[1];
	m_camera.Xiw[2][2] = Zvector[2];
	m_camera.Xiw[2][3] = -(Zvector[X] * m_camera.position[X] + Zvector[Y] * m_camera.position[Y] + Zvector[Z] * m_camera.position[Z]);

	m_camera.Xiw[3][0] = 0.0f;
	m_camera.Xiw[3][1] = 0.0f;
	m_camera.Xiw[3][2] = 0.0f;
	m_camera.Xiw[3][3] = 1.0f;

	/* Compute Xpi */
	resetMatrix(m_camera.Xpi);
	m_camera.Xpi[0][0] = 1.0f;
	m_camera.Xpi[1][1] = 1.0f;
	m_camera.Xpi[2][2] = (float)tan(dtor(m_camera.FOV / 2.0f));
	m_camera.Xpi[3][3] = 1.0f;
	m_camera.Xpi[3][2] = (float)tan(dtor(m_camera.FOV / 2.0f));

	/* Push onto stack in Xsp -> Xpi -> Xiw to make Xsw*/
	GzPushMatrix(Xsp);
	GzPushMatrix(m_camera.Xpi);
	GzPushMatrix(m_camera.Xiw);
	return GZ_SUCCESS;
}

int GzRender::GzPutCamera(GzCamera camera)
{
/* HW 3.8 
/*- overwrite renderer camera structure with new camera definition
*/
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			m_camera.Xiw[i][j] = camera.Xiw[i][j];
			m_camera.Xpi[i][j] = camera.Xpi[i][j];
		}
	}
	for (int i = 0; i < 3; i++) {
		m_camera.position[i] = camera.position[i];
		m_camera.lookat[i] = camera.lookat[i];
		m_camera.worldup[i] = camera.worldup[i];
	}
	m_camera.FOV = camera.FOV;
	return GZ_SUCCESS;	
}

int GzRender::GzPushMatrix(GzMatrix	matrix)
{
/* HW 3.9 
- push a matrix onto the Ximage stack
- check for stack overflow
*/
	GzMatrix identity = { {1.0f, 0.0f, 0.0f, 0.0f},
						  {0.0f, 1.0f, 0.0f, 0.0f},
						  {0.0f, 0.0f, 1.0f, 0.0f},
						  {0.0f, 0.0f, 0.0f, 1.0f} };


	if (matlevel < MATLEVELS) { /* Check for overflow */

		/* For the normal vectors stack */
		if (matlevel < 2) {	/* To push Xsp & Xpi onto normal stack */
			GzPushNormMatrix(identity);
		}
		else if (matlevel == 2) {	/* Push Xiw onto normal stack */
			GzMatrix normXiw;
			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					normXiw[i][j] = m_camera.Xiw[i][j];
				}
			}
			normXiw[0][3] = 0.0f;
			normXiw[1][3] = 0.0f;
			normXiw[2][3] = 0.0f;

			GzPushNormMatrix(normXiw);
		}
		else {	/* Make sure we only push rotational matrices onto stack */
			if (matrix[0][1] == 0.0f && matrix[0][2] == 0.0f && matrix[1][0] == 0.0f
				&& matrix[1][2] == 0.0f && matrix[2][0] == 0.0f && matrix[2][1] == 0.0f) {
				GzPushNormMatrix(identity);
			}
			else {
				GzPushNormMatrix(matrix);
			}
		} /* End of normal vectors stack */

		/* For usual transformation matrices stack */
		if (matlevel == -1) {
			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					Ximage[0][i][j] = matrix[i][j];
				}
			}
		}
		else {
			/* Push the matrix product to top of stack */
			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					Ximage[matlevel + 1][i][j] = 0.0f;
					for (int k = 0; k < 4; k++) {
						Ximage[matlevel + 1][i][j] += Ximage[matlevel][i][k] * matrix[k][j];
					}
				}
			}
		}
		matlevel++;
	}
	else return GZ_FAILURE;
	return GZ_SUCCESS;
}

int GzRender::GzPopMatrix()
{
/* HW 3.10
- pop a matrix off the Ximage stack
- check for stack underflow
*/

	/* Decrement the counter */
	if (matlevel >= 0) {
		matlevel--;
	}
	return GZ_SUCCESS;
}

int GzRender::GzPut(int i, int j, GzIntensity r, GzIntensity g, GzIntensity b, GzIntensity a, GzDepth z)
{
/* HW1.4 write pixel values into the buffer */
	if (i >= 0 && i < xres && j >= 0 && j < yres) {
		//Do bounds checking
		if (r < 0) r = 0;
		if (r > C_LIMIT) r = C_LIMIT;
		if (g < 0) g = 0;
		if (g > C_LIMIT) g = C_LIMIT;
		if (b < 0) b = 0;
		if (b > C_LIMIT) b = C_LIMIT;
		if (z < pixelbuffer[ARRAY(i, j)].z) {
			pixelbuffer[ARRAY(i, j)] = { r, g, b, a, z };
		}
	}
	return GZ_SUCCESS;
}


int GzRender::GzGet(int i, int j, GzIntensity *r, GzIntensity *g, GzIntensity *b, GzIntensity *a, GzDepth *z)
{
/* HW1.5 retrieve a pixel information from the pixel buffer */
	if (i >= 0 && i < xres && j >= 0 && j < yres) {
		*r = pixelbuffer[ARRAY(i, j)].red;
		*g = pixelbuffer[ARRAY(i, j)].green;
		*b = pixelbuffer[ARRAY(i, j)].blue;
		*a = pixelbuffer[ARRAY(i, j)].alpha;
		*z = pixelbuffer[ARRAY(i, j)].z;
	}
	return GZ_SUCCESS;
}


int GzRender::GzFlushDisplay2File(FILE* outfile)
{
/* HW1.6 write image to ppm file -- "P6 %d %d 255\r" */
	fprintf(outfile, "P6 %d %d 255\r", xres, yres);
	int res = xres * yres;
	for (int i = 0; i < res; i++) {

		//Keep the 8 least significant bits and put it in a buffer for fwrite
		char buffer[3] = { (char)(pixelbuffer[i].red >> 4), (char)(pixelbuffer[i].green >> 4), (char)(pixelbuffer[i].blue >> 4) };
		fwrite(buffer, 1, 3, outfile);
	}
	return GZ_SUCCESS;
}

int GzRender::GzFlushDisplay2FrameBuffer()
{
/* HW1.7 write pixels to framebuffer: 
	- put the pixels into the frame buffer
	- CAUTION: when storing the pixels into the frame buffer, the order is blue, green, and red 
	- NOT red, green, and blue !!!
*/
	int res = xres * yres;
	for (int i = 0; i < res; i++) {
		framebuffer[3 * i] = (char)(pixelbuffer[i].blue >> 4);
		framebuffer[3 * i + 1] = (char)(pixelbuffer[i].green >> 4);
		framebuffer[3 * i + 2] = (char)(pixelbuffer[i].red >> 4);
	}
	return GZ_SUCCESS;
}


/***********************************************/
/* HW2 methods: implement from here */

int GzRender::GzPutAttribute(int numAttributes, GzToken	*nameList, GzPointer *valueList) 
{
/* HW 2.1
-- Set renderer attribute states (e.g.: GZ_RGB_COLOR default color)
-- In later homeworks set shaders, interpolaters, texture maps, and lights
*/

/*
- GzPutAttribute() must accept the following tokens/values:

- GZ_RGB_COLOR					GzColor		default flat-shade color
- GZ_INTERPOLATE				int			shader interpolation mode
- GZ_DIRECTIONAL_LIGHT			GzLight
- GZ_AMBIENT_LIGHT            	GzLight		(ignore direction)
- GZ_AMBIENT_COEFFICIENT		GzColor		Ka reflectance
- GZ_DIFFUSE_COEFFICIENT		GzColor		Kd reflectance
- GZ_SPECULAR_COEFFICIENT       GzColor		Ks coef's
- GZ_DISTRIBUTION_COEFFICIENT   float		spec power
- GZ_TEXTURE_MAP				GzTexture	texture
- GZ_AASHIFTX					float		x-shift
- GZ_AASHIFTY					float		y-shift
*/
	for (int i = 0; i < numAttributes; i++) {
		GzToken token = nameList[i];
		if (token == GZ_RGB_COLOR) {
			GzColor* color = (GzColor*)valueList[i];
			for (int j = 0; j < 3; j++) {
				flatcolor[j] = (*color)[j];
			}
		}
		else if (token == GZ_INTERPOLATE) {
			int* shading = (int*)valueList[i];
			interp_mode = *shading;
		}
		else if (token == GZ_DIRECTIONAL_LIGHT) {
			GzLight* light = (GzLight*)valueList[i];
			for (int j = 0; j < 3; j++) {
				lights[numlights].direction[j] = light->direction[j];
				lights[numlights].color[j] = light->color[j];
			}
			numlights++;
		}
		else if (token == GZ_AMBIENT_LIGHT) {
			GzLight* ambience = (GzLight*)valueList[i];
			for (int j = 0; j < 3; j++) {
				ambientlight.direction[j] = ambience->direction[j];
				ambientlight.color[j] = ambience->color[j];
			}
		}
		else if (token == GZ_AMBIENT_COEFFICIENT) {
			GzColor* amb = (GzColor*)valueList[i];
			for (int j = 0; j < 3; j++) {
				Ka[j] = (*amb)[j];
			}
		}
		else if (token == GZ_DIFFUSE_COEFFICIENT) {
			GzColor* diff = (GzColor*)valueList[i];
			for (int j = 0; j < 3; j++) {
				Kd[j] = (*diff)[j];
			}
		}
		else if (token == GZ_SPECULAR_COEFFICIENT) {
			GzColor* spe = (GzColor*)valueList[i];
			for (int j = 0; j < 3; j++) {
				Ks[j] = (*spe)[j];
			}
		}
		else if (token == GZ_DISTRIBUTION_COEFFICIENT) {
			float* sp = (float*)valueList[i];
			spec = *sp;
		}
		else if (token == GZ_TEXTURE_MAP) {
			GzTexture tex = (GzTexture)valueList[i];
			if (tex) tex_fun = tex;
		}
		else if (token == GZ_AASHIFTX) {
			float* shift = (float*)valueList[i];
			offset[X] = *shift;
		}
		else if (token == GZ_AASHIFTY) {
			float* shift = (float*)valueList[i];
			offset[Y] = *shift;
		}
	}
	return GZ_SUCCESS;
}

int GzRender::GzPutTriangle(int numParts, GzToken *nameList, GzPointer *valueList)
/* numParts - how many names and values */
{
/* HW 2.2
-- Pass in a triangle description with tokens and values corresponding to
      GZ_NULL_TOKEN:		do nothing - no values
      GZ_POSITION:		3 vert positions in model space
-- Return error code
*/
/*
-- Xform positions of verts using matrix on top of stack 
-- Clip - just discard any triangle with any vert(s) behind view plane 
		- optional: test for triangles with all three verts off-screen (trivial frustum cull)
-- invoke triangle rasterizer  
*/
	GzCoord vertices[3], normals[3];					/* Vertices and normals */
	GzColor diff[3], amb[3], specu[3];					/* Light */
	GzColor colorC[3];									/* colorC[ vertex ][RED | GREEN | BLUE ]*/
	GzTextureIndex textures[3];									/* to store uv values */

	float convert4D[3][4], normals4D[3][4];						/* HW 4*/
	float transformed[3][4], transformednormals[3][4];			/* HW 4*/

	/* store vertice coordinates and normal coordinates */
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			vertices[i][j] = ((GzCoord*)valueList[0])[i][j];			/* HW1 Store vertex coordinates */
			normals[i][j] = ((GzCoord*)valueList[1])[i][j];				/* HW4 Store normal coordinates */
			if (j < 2) textures[i][j] = ((GzTextureIndex*)valueList[2])[i][j];	/* HW 5 Store uv values */
		}
	}

	/* Convert 3D to 4D by adding w = 1 */
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			convert4D[i][j] = vertices[i][j];
			normals4D[i][j] = normals[i][j];					/* HW 4*/
		}
		convert4D[i][3] = 1.0f;
		normals4D[i][3] = 1.0f;									/* HW 4*/
	}

	/* Multiply top of stack and new 4D vertex */
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 4; j++) {
			transformed[i][j] = 0.0f;
			transformednormals[i][j] = 0.0f;					/* HW 4*/
			for (int k = 0; k < 4; k++) {
				transformed[i][j] += Ximage[matlevel][j][k] * convert4D[i][k];
				transformednormals[i][j] += Xnorm[matlevel][j][k] * normals4D[i][k];	/* HW 4*/
			}
		}
	}

	/* Then convert 4D coordinates to 3D */
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			vertices[i][j] = transformed[i][j] / transformed[i][3];
			normals[i][j] = transformednormals[i][j] / transformednormals[i][3];		/* HW 4*/
		}

		// HW6 Adjust for any offsets
		vertices[i][X] -= offset[X];
		vertices[i][Y] -= offset[Y];
	}

	/* Sort by ascending y-values */
	sort(vertices, normals, textures);

	/* Now find L/R relationship */
	if (vertices[0][1] == vertices[1][1]) { /* Top edge is horizontal */
		if (vertices[0][0] > vertices[1][0]) {
			std::swap(vertices[1], vertices[2]);
			std::swap(normals[1] , normals[2]);
			std::swap(textures[1], textures[2]);
		}
	}
	else if (vertices[1][1] == vertices[2][1]) { /* Bottom edge is horizontal */
		if (vertices[2][0] > vertices[1][0]) {
			std::swap(vertices[1], vertices[2]);
			std::swap(normals[1] , normals[2]);
			std::swap(textures[1], textures[2]);
		}
	}
	/* no horizontal edge, find Xp */
	else {
		float dX = vertices[2][0] - vertices[0][0];
		float dY = vertices[2][1] - vertices[0][1];
		float A = dY;
		float B = -dX;
		float C = A * vertices[1][0] + B * vertices[1][1];
		float Xp = (B * vertices[1][1] - C) / A;
		if (Xp > vertices[1][0]) {
			std::swap(vertices[1], vertices[2]);
			std::swap(normals[1] , normals[2]);
			std::swap(textures[1], textures[2]);
		}
	}

	/* Form edge equations */
	/* edge 12*/
	float dX12 = vertices[1][0] - vertices[0][0];
	float dY12 = vertices[1][1] - vertices[0][1];
	float A12 = dY12;
	float B12 = -dX12;
	float C12 = dX12 * vertices[0][1] - dY12 * vertices[0][0];

	/* Edge 23*/
	float dX23 = vertices[2][0] - vertices[1][0];
	float dY23 = vertices[2][1] - vertices[1][1];
	float A23 = dY23;
	float B23 = -dX23;
	float C23 = dX23 * vertices[1][1] - dY23 * vertices[1][0];

	/* Edge 31*/
	float dX31 = vertices[0][0] - vertices[2][0];
	float dY31 = vertices[0][1] - vertices[2][1];
	float A31 = dY31;
	float B31 = -dX31;
	float C31 = dX31 * vertices[2][1] - dY31 * vertices[2][0];

	/* Choose two pairs of vertices to make edge vertices */
	float X1 = vertices[1][0] - vertices[0][0];
	float Y1 = vertices[1][1] - vertices[0][1];
	float Z1 = vertices[1][2] - vertices[0][2];

	float X2 = vertices[2][0] - vertices[1][0];
	float Y2 = vertices[2][1] - vertices[1][1];
	float Z2 = vertices[2][2] - vertices[1][2];

	float A = (Y1 * Z2) - (Z1 * Y2);
	float B = (X2 * Z1) - (X1 * Z2);
	float C = (X1 * Y2) - (Y1 * X2);
	float D = -(A * vertices[0][0] + B * vertices[0][1] + C * vertices[0][2]);

	/* Set lighting for each vertex */
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			diff[i][j] = 0.0f;
			amb[i][j] = 0.0f;
			specu[i][j] = 0.0f;
			colorC[i][j] = 0.0f;
		}
	}

	/* Texture for Gouraud Shading */
	if (interp_mode == GZ_COLOR) {
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < numlights; j++) {
				GzCoord R, E;			/* Reflection and view-point (eye) */

				/* Default E */
				E[0] = 0.0f;
				E[1] = 0.0f;
				E[2] = -1.0f;

				float NDotL = normals[i][0] * lights[j].direction[0] + normals[i][1] * lights[j].direction[1] + normals[i][2] * lights[j].direction[2];
				float NDotE = normals[i][0] * E[0] + normals[i][1] * E[1] + normals[i][2] * E[2];

				if (NDotL * NDotE > 0) {
					for (int k = 0; k < 3; k++) {
						R[k] = 2.0f * NDotL * normals[i][k] - lights[j].direction[k];
					}

					/* Calculate magnitude of R vector */
					float magR = (float)sqrt(R[0] * R[0] + R[1] * R[1] + R[2] * R[2]);

					/* Normalize reflection vector */
					for (int k = 0; k < 3; k++) {
						R[k] /= magR;
					}

					/* Dot R with E */
					for (int k = 0; k < 3; k++) {
						float RDotE = R[0] * E[0] + R[1] * E[1] + R[2] * E[2];
						/* Clamp it to 0 or 1 if under or over */
						if (RDotE < 0) {
							RDotE = 0.0f;
						}
						if (RDotE > 1.0f) {
							RDotE = 1.0f;
						}

						/* If there is texture */
						if (tex_fun) {
							Ks[k] = 1.0f;
							Kd[k] = 1.0f;
							Ka[k] = 1.0f;
						}
						
						/* Calculate specular component */
						specu[i][k] += Ks[k] * (float)pow(RDotE, spec) * lights[j].color[k];

						/* Calculate diffuse component */
						if (NDotL > 0 && NDotE > 0) {
							diff[i][k] += Kd[k] * (normals[i][0] * lights[j].direction[0]
								+ normals[i][1] * lights[j].direction[1]
								+ normals[i][2] * lights[j].direction[2]) * lights[j].color[k];

						}
						else {
							diff[i][k] += Kd[k] * (-normals[i][0] * lights[j].direction[0]
								- normals[i][1] * lights[j].direction[1]
								- normals[i][2] * lights[j].direction[2]) * lights[j].color[k];
						}
					}
				}
			}
			/* Calculate ambient component */
			for (int j = 0; j < 3; j++) {
				amb[i][j] += Ka[j] * ambientlight.color[j];
			} // End j 
		} // End i

		/* Calculate Color C */
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {

				colorC[i][j] = specu[i][j] + diff[i][j] + amb[i][j];

				/* Clamp values to [0, 1] */
				if (colorC[i][j] < 0.0f) colorC[i][j] = 0.0f;
				if (colorC[i][j] > 1.0f) colorC[i][j] = 1.0f;
			} // End j loop
		} // End i loop / color C calculation
	} // End if Gourad Shading

	/* ----- Interpolate colors ----- */
	GzCoord red12 = { vertices[1][X] - vertices[0][X], vertices[1][Y] - vertices[0][Y], colorC[1][RED] - colorC[0][RED] };
	GzCoord red13 = { vertices[2][X] - vertices[0][X], vertices[2][Y] - vertices[0][Y], colorC[2][RED] - colorC[0][RED] };
	float redA =   red12[Y] * red13[Z] - red12[Z] * red13[Y];
	float redB = -(red12[X] * red13[Z] - red12[Z] * red13[X]);
	float redC =   red12[X] * red13[Y] - red12[Y] * red13[X];
	float redD = -(redA * vertices[0][X] + redB * vertices[0][Y] + redC * colorC[0][RED]);

	GzCoord green12 = { vertices[1][X] - vertices[0][X], vertices[1][Y] - vertices[0][Y], colorC[1][GREEN] - colorC[0][GREEN] };
	GzCoord green13 = { vertices[2][X] - vertices[0][X], vertices[2][Y] - vertices[0][Y], colorC[2][GREEN] - colorC[0][GREEN] };
	float greenA =   green12[Y] * green13[Z] - green12[Z] * green13[Y];
	float greenB = -(green12[X] * green13[Z] - green12[Z] * green13[X]);
	float greenC =   green12[X] * green13[Y] - green12[Y] * green13[X];
	float greenD = -(greenA * vertices[0][X] + greenB * vertices[0][Y] + greenC * colorC[0][GREEN]);

	GzCoord blue12 = { vertices[1][X] - vertices[0][X], vertices[1][Y] - vertices[0][Y], colorC[1][BLUE] - colorC[0][BLUE] };
	GzCoord blue13 = { vertices[2][X] - vertices[0][X], vertices[2][Y] - vertices[0][Y], colorC[2][BLUE] - colorC[0][BLUE] };
	float blueA =   blue12[Y] * blue13[Z] - blue12[Z] * blue13[Y];
	float blueB = -(blue12[X] * blue13[Z] - blue12[Z] * blue13[X]);
	float blueC =   blue12[X] * blue13[Y] - blue12[Y] * blue13[X];
	float blueD = -(blueA * vertices[0][X] + blueB * vertices[0][Y] + blueC * colorC[0][BLUE]);
	/* ----- End of color interpolation -----*/


	/* ----- Interpolate normals -----*/
	GzCoord normalsX12 = { vertices[1][X] - vertices[0][X], vertices[1][Y] - vertices[0][Y], normals[1][X] - normals[0][X] };
	GzCoord normalsX13 = { vertices[2][X] - vertices[0][X], vertices[2][Y] - vertices[0][Y], normals[2][X] - normals[0][X] };
	float normalsXA =   normalsX12[Y] * normalsX13[Z] - normalsX12[Z] * normalsX13[Y];
	float normalsXB = -(normalsX12[X] * normalsX13[Z] - normalsX12[Z] * normalsX13[X]);
	float normalsXC =   normalsX12[X] * normalsX13[Y] - normalsX12[Y] * normalsX13[X];
	float normalsXD = -(normalsXA * vertices[0][X] + normalsXB * vertices[0][Y] + normalsXC * normals[0][X]);

	GzCoord normalsY12 = { vertices[1][X] - vertices[0][X], vertices[1][Y] - vertices[0][Y], normals[1][Y] - normals[0][Y] };
	GzCoord normalsY13 = { vertices[2][X] - vertices[0][X], vertices[2][Y] - vertices[0][Y], normals[2][Y] - normals[0][Y] };
	float normalsYA =   normalsY12[Y] * normalsY13[Z] - normalsY12[Z] * normalsY13[Y];
	float normalsYB = -(normalsY12[X] * normalsY13[Z] - normalsY12[Z] * normalsY13[X]);
	float normalsYC =   normalsY12[X] * normalsY13[Y] - normalsY12[Y] * normalsY13[X];
	float normalsYD = -(normalsYA * vertices[0][X] + normalsYB * vertices[0][Y] + normalsYC * normals[0][Y]);

	GzCoord normalsZ12 = { vertices[1][X] - vertices[0][X], vertices[1][Y] - vertices[0][Y], normals[1][Z] - normals[0][Z] };
	GzCoord normalsZ13 = { vertices[2][X] - vertices[0][X], vertices[2][Y] - vertices[0][Y], normals[2][Z] - normals[0][Z] };
	float normalsZA =   normalsZ12[Y] * normalsZ13[Z] - normalsZ12[Z] * normalsZ13[Y];
	float normalsZB = -(normalsZ12[X] * normalsZ13[Z] - normalsZ12[Z] * normalsZ13[X]);
	float normalsZC =   normalsZ12[X] * normalsZ13[Y] - normalsZ12[Y] * normalsZ13[X];
	float normalsZD = -(normalsZA * vertices[0][X] + normalsZB * vertices[0][Y] + normalsZC * normals[0][Z]);
	/* ----- End of normals interpolation -----*/

	/* Perspective Correction */
	GzTextureIndex perspTexture[3];
	for (int i = 0; i < 3; i++) {
		float VzPrime = vertices[i][2] / ((float)MAXINT - vertices[i][2]);
		perspTexture[i][U] = textures[i][U] / (VzPrime + 1.0f);
		perspTexture[i][V] = textures[i][V] / (VzPrime + 1.0f);
	}

	/* ----- Interpolate U V values ----- */
	GzCoord u12 = { vertices[1][X] - vertices[0][X], vertices[1][Y] - vertices[0][Y], perspTexture[1][U] - perspTexture[0][U] };
	GzCoord u13 = { vertices[2][X] - vertices[0][X], vertices[2][Y] - vertices[0][Y], perspTexture[2][U] - perspTexture[0][U] };
	float uA =   u12[Y] * u13[Z] - u12[Z] * u13[Y];
	float uB = -(u12[X] * u13[Z] - u12[Z] * u13[X]);
	float uC =   u12[X] * u13[Y] - u12[Y] * u13[X];
	float uD = -(uA * vertices[0][X] + uB * vertices[0][Y] + uC * perspTexture[0][U]);
	
	GzCoord v12 = { vertices[1][X] - vertices[0][X], vertices[1][Y] - vertices[0][Y], perspTexture[1][V] - perspTexture[0][V] };
	GzCoord v13 = { vertices[2][X] - vertices[0][X], vertices[2][Y] - vertices[0][Y], perspTexture[2][V] - perspTexture[0][V] };
	float vA =   v12[Y] * v13[Z] - v12[Z] * v13[Y];
	float vB = -(v12[X] * v13[Z] - v12[Z] * v13[X]);
	float vC =   v12[X] * v13[Y] - v12[Y] * v13[X];
	float vD = -(vA * vertices[0][X] + vB * vertices[0][Y] + vC * perspTexture[0][V]);
	/* ----- End of U V interpolation*/

	/* Set bounds */
	float min_x = min(min(vertices[0][0], vertices[1][0]), vertices[2][0]);
	float max_x = max(max(vertices[0][0], vertices[1][0]), vertices[2][0]);

	float min_y = min(min(vertices[0][1], vertices[1][1]), vertices[2][1]);
	float max_y = max(max(vertices[0][1], vertices[1][1]), vertices[2][1]);

	/* Check bounds */
	if (min_x < 0) min_x = 0;
	if (max_x >= xres) max_x = xres - 1.0f;
	if (min_y < 0) min_y = 0;
	if (max_y >= yres) max_y = yres - 1.0f;

	/* The bounding box for our pixels */
	int bounded_min_x = (int)floor(min_x);
	int bounded_max_x = (int)ceil(max_x);
	int bounded_min_y = (int)floor(min_y);
	int bounded_max_y = (int)ceil(max_y);

	for (int i = bounded_min_x; i <= bounded_max_x; i++) {
		for (int j = bounded_min_y; j <= bounded_max_y; j++) {
			float LEE12 = A12 * i + B12 * j + C12;
			float LEE23 = A23 * i + B23 * j + C23;
			float LEE31 = A31 * i + B31 * j + C31;

			if ((LEE12 > 0 && LEE23 > 0 && LEE31 > 0) || (LEE12 < 0 && LEE23 < 0 && LEE31 < 0)) {

				/* Check to make sure we don't get a divide-by-zero situation */
				if (C != 0.0f && redC != 0 && greenC != 0 && blueC != 0.0f
					&& normalsXC != 0.0f && normalsYC != 0.0f && normalsZC != 0.0f
					&& uC != 0.0f && vC != 0.0f) {
					GzDepth current_z = (GzDepth)(-(A * i + B * j + D) / C);
					GzIntensity r = 0, g = 0, b = 0;
					if (current_z >= 0) {	/* Ensure z-value is not passed the screen */
						if (interp_mode == GZ_COLOR) {	/* Gouraud Shading */
							/* interpolate color */
							float tempred = -(redA * i + redB * j + redD) / redC;
							float tempblue = -(blueA * i + blueB * j + blueD) / blueC;
							float tempgreen = -(greenA * i + greenB * j + greenD) / greenC;

							if (tex_fun) {
								GzTextureIndex current_uv; 
								GzColor uvcolor;
								float interpolate_VzPrime = (float)current_z / ((float)MAXINT - (float)current_z);
								current_uv[U] = -(uA * i + uB * j + uD) / uC;
								current_uv[V] = -(vA * i + vB * j + vD) / vC;

								current_uv[U] *= interpolate_VzPrime + 1.0f;
								current_uv[V] *= interpolate_VzPrime + 1.0f;

								tex_fun(current_uv[U], current_uv[V], uvcolor);
								
								tempred *= uvcolor[RED];
								tempgreen *= uvcolor[GREEN];
								tempblue *= uvcolor[BLUE];
							}

							r = ctoi(tempred);
							b = ctoi(tempblue);
							g = ctoi(tempgreen);
						}

						else if (interp_mode == GZ_NORMALS) {		/* Phong Shading */
							/* Interpolate color */
							GzColor intensity;
							GzCoord interpolatedNormal;
							interpolatedNormal[X] = -(normalsXA * i + normalsXB * j + normalsXD) / normalsXC;
							interpolatedNormal[Y] = -(normalsYA * i + normalsYB * j + normalsYD) / normalsYC;
							interpolatedNormal[Z] = -(normalsZA * i + normalsZB * j + normalsZD) / normalsZC;

							if (tex_fun) {
								GzTextureIndex current_uv;
								GzColor uvcolor;
								float interpolate_VzPrime = (float)current_z / ((float)MAXINT - (float)current_z);
								current_uv[U] = -(uA * i + uB * j + uD) / uC;
								current_uv[V] = -(vA * i + vB * j + vD) / vC;
								
								current_uv[U] *= interpolate_VzPrime + 1.0f;
								current_uv[V] *= interpolate_VzPrime + 1.0f;

								tex_fun(current_uv[U], current_uv[V], uvcolor);
								
								for (int color = 0; color < 3; color++) {
									Kd[color] = uvcolor[color];
									Ka[color] = uvcolor[color];
								}
							}

							GzColor specIntensity = { 0.0f, 0.0f, 0.0f }, diffIntensity = { 0.0f, 0.0f, 0.0f }, ambiIntensity = { 0.0f, 0.0f, 0.0f };

							for (int nl = 0; nl < numlights; nl++) {	/* Using nl for num of lights. Losing track of iterators */
								GzCoord R, E = { 0.0f, 0.0f, -1.0f };
								float NDotL = interpolatedNormal[0] * lights[nl].direction[0] + interpolatedNormal[1] * lights[nl].direction[1] + interpolatedNormal[2] * lights[nl].direction[2];
								float NDotE = interpolatedNormal[0] * E[0] + interpolatedNormal[1] * E[1] + interpolatedNormal[2] * E[2];
								if (NDotL * NDotE > 0) {
									for (int l = 0; l < 3; l++) {
										R[l] = 2.0f * NDotL * interpolatedNormal[l] - lights[nl].direction[l];
									}
									float magR = (float)sqrt(R[0] * R[0] + R[1] * R[1] + R[2] * R[2]);
									for (int l = 0; l < 3; l++) {
										R[l] /= magR;
									}
									for (int l = 0; l < 3; l++) {
										float RDotE = R[0] * E[0] + R[1] * E[1] + R[2] * E[2];
										/* Clamp values to 0.0f or 1.0f if under or over */
										if (RDotE < 0.0f) RDotE = 0.0f;
										if (RDotE > 1.0f) RDotE = 1.0f;
										
										specIntensity[l] += Ks[l] * (float)pow(RDotE, spec) * lights[nl].color[l];

										/* We only care about cases where NDotL and NDotE are both positive or both negative */
										/* If NDotL and NDotE are opposite signs, that means light and eye are on opposite sides of the surface. SKIP IT */
										if (NDotL > 0.0f && NDotE > 0.0f) {	/* Both positive */
											diffIntensity[l] += Kd[l] * (interpolatedNormal[X] * lights[nl].direction[X]
												+ interpolatedNormal[Y] * lights[nl].direction[Y]
												+ interpolatedNormal[Z] * lights[nl].direction[Z]) * lights[nl].color[l];
										}
										else if (NDotL < 0.0f && NDotE < 0.0f) { /* Both negative */
											diffIntensity[l] += Kd[l] * (-interpolatedNormal[X] * lights[nl].direction[X]
												- interpolatedNormal[Y] * lights[nl].direction[Y]
												- interpolatedNormal[Z] * lights[nl].direction[Z]) * lights[nl].color[l];
										}
									}
								}
							}
							for (int k = 0; k < 3; k++) {
								ambiIntensity[k] += Ka[k] * ambientlight.color[k];
							}

							for (int k = 0; k < 3; k++) {
								intensity[k] = specIntensity[k] + diffIntensity[k] + ambiIntensity[k];
								if (intensity[k] < 0.0f) intensity[k] = 0.0f;
								if (intensity[k] > 1.0f) intensity[k] = 1.0f;
							}
							r = ctoi(intensity[RED]);
							g = ctoi(intensity[GREEN]);
							b = ctoi(intensity[BLUE]);
						}

						else {	/* If no defined shading, do flat shading */
							r = ctoi(flatcolor[RED]);
							g = ctoi(flatcolor[GREEN]);
							b = ctoi(flatcolor[BLUE]);
						}
					}
					GzPut(i, j, r, g, b, 1, current_z);
				}
			}
		}
	}
	return GZ_SUCCESS;
}

/* sort function to sort GzCoord[3][3] by the x, y, or z axis */
/* sorts from least to greatest */
void GzRender::sort(GzCoord *v, GzCoord* norms, GzTextureIndex* textures) {
	//compare v[0] with v[1], v[0] with v[2], and v[1] with v[2]
	for (int i = 0; i < 2; i++) {
		for (int j = i + 1; j < 3; j++) { /* 01, 02, 12 */
			if (v[i][1] > v[j][1]) {
				std::swap(v[i], v[j]);
				std::swap(norms[i], norms[j]);
				std::swap(textures[i], textures[j]);
			}
		}
	}
}

/*
void GzRender::swap(GzCoord v1, GzCoord v2) {
	for (int i = 0; i < 3; i++) {
		float temp = v1[i];
		v1[i] = v2[i];
		v2[i] = temp;
	}
}

void GzRender::swap(GzTextureIndex first, GzTextureIndex second) {
	for (int i = 0; i < 2; i++) {
		float temp = first[i];
		first[i] = second[i];
		second[i] = temp;
	}
}
*/

float GzRender::dtor(float degree) {
	return degree * PI / 180.0f;
}

void GzRender::resetMatrix(GzMatrix m) {
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			m[i][j] = 0.0f;
		}
	}
}

int GzRender::GzPushNormMatrix(GzMatrix norm) {
	if (matlevel == -1) {
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				Xnorm[0][i][j] = norm[i][j];
			}
		}
	}
	else {
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				Xnorm[matlevel + 1][i][j] = 0.0f;
				for (int k = 0; k < 4; k++) {
					Xnorm[matlevel + 1][i][j] += Xnorm[matlevel][i][k] * norm[k][j];
				}
			}
		}
	}
	return GZ_SUCCESS;
}
