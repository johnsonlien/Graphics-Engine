/* CS580 Homework 3 */

#include	"stdafx.h"
#include	"stdio.h"
#include	"math.h"
#include	"Gz.h"
#include	"rend.h"

#define PI (float) 3.14159265358979323846

int GzRender::GzRotXMat(float degree, GzMatrix mat)
{
/* HW 3.1
// Create rotate matrix : rotate along x axis
// Pass back the matrix using mat value
*/
	/* convert degrees to radians */
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

	/* Setup Xsp matrix */
	resetMatrix(Xsp);
	Xsp[0][0] = (float)xRes / 2.0f;
	Xsp[0][3] = (float)xRes / 2.0f;
	Xsp[1][1] = (float)-yRes / 2.0f;
	Xsp[1][3] = (float)yRes / 2.0f;
	Xsp[2][2] = (float)INT_MAX;
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
		pixelbuffer[i] = { C_LIMIT, 2000, 3000, 1, INT_MAX };
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
	/* Initialize frame buffer */
	
	/* Compute Xiw*/
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
	
	if (matlevel < MATLEVELS) { /* Check for overflow */
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
	for (int i = 0; i < numAttributes; i++) {
		if (nameList[i] == GZ_RGB_COLOR) {
			GzColor* color = (GzColor*)valueList[i];
			flatcolor[0] = (*color)[0];
			flatcolor[1] = (*color)[1];
			flatcolor[2] = (*color)[2];
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
-- Invoke the rastrizer/scanline framework
-- Return error code
*/

	for (int np = 0; np < numParts; np++) {
		GzCoord vertices[3];

		float convert4D[3][4];
		float transformed[3][4];

		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				vertices[i][j] = ((GzCoord*)valueList[np])[i][j];
			}
		}

		/* Convert 3D to 4D by adding w = 1 */
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				convert4D[i][j] = vertices[i][j];
			}
			convert4D[i][3] = 1.0f;
		}

		/* Multiply top of stack and new 4D vertex */
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 4; j++) {
				transformed[i][j] = 0.0f;
				for (int k = 0; k < 4; k++) {
					transformed[i][j] += Ximage[matlevel][j][k] * convert4D[i][k];
				}
			}
		}
		/* Then convert 4D coordinates to 3D */
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				vertices[i][j] = transformed[i][j] / transformed[i][3];
			}
		}
		/* Sort by ascending y-values */
		sort(vertices);

		/* Now find L/R relationship */
		if (vertices[0][1] == vertices[1][1]) { /* Top edge is horizontal */
			if (vertices[0][0] > vertices[1][0]) {
				swap(vertices[1], vertices[2]);
			}
		}
		else if (vertices[1][1] == vertices[2][1]) { /* Bottom edge is horizontal */
			if (vertices[2][0] > vertices[1][0]) {
				swap(vertices[1], vertices[2]);
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
				swap(vertices[1], vertices[2]);
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
					if (C != 0.0) {
						GzDepth current_z = (GzDepth)(-(A * i + B * j + D) / C);
						GzPut(i, j, ctoi(flatcolor[0]), ctoi(flatcolor[1]), ctoi(flatcolor[2]), 1, current_z);
					}
				}
			}
		}
	}
	return GZ_SUCCESS;
}

/* sort function to sort GzCoord[3][3] by the x, y, or z axis */
/* sorts from least to greatest */
void GzRender::sort(GzCoord *v) {
	//compare v[0] with v[1], v[0] with v[2], and v[1] with v[2]
	for (int i = 0; i < 2; i++) {
		for (int j = i + 1; j < 3; j++) {
			if (v[i][1] > v[j][1]) {
				swap(v[i], v[j]);
			}
		}
	}
}

void GzRender::swap(GzCoord v1, GzCoord v2) {
	for (int i = 0; i < 3; i++) {
		float temp = v1[i];
		v1[i] = v2[i];
		v2[i] = temp;
	}
}

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