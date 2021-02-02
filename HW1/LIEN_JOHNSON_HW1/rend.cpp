#include	"stdafx.h"
#include	"stdio.h"
#include	"math.h"
#include	"Gz.h"
#include	"rend.h"
/*   CS580 HW   */
#include    "stdafx.h"  
#include	"Gz.h"

#define C_LIMIT 4095		//12-bit limit for colors

GzRender::GzRender(int xRes, int yRes)
{
/* HW1.1 create a framebuffer for MS Windows display:
 -- set display resolution
 -- allocate memory for framebuffer : 3 bytes(b, g, r) x width x height
 -- allocate memory for pixel buffer
 */
	xres = (unsigned short)xRes;
	yres = (unsigned short)yRes; 

	framebuffer = new char[xres * yres * 3];
	pixelbuffer = new GzPixel[xres * yres];
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
		//				   red		green blue  a  z
		pixelbuffer[i] = { C_LIMIT, 2000, 3000, 1, 0 };
	}
	return GZ_SUCCESS;
}


int GzRender::GzPut(int i, int j, GzIntensity r, GzIntensity g, GzIntensity b, GzIntensity a, GzDepth z)
{
/* HW1.4 write pixel values into the buffer */

	//Make sure x,y coordinates are within our resolution
	if (i >= 0 && i < xres && j >= 0 && j < yres) {
		//Do bounds checking
		if (r < 0) r = 0;
		if (r > C_LIMIT) r = C_LIMIT;
		if (g < 0) g = 0;
		if (g > C_LIMIT) g = C_LIMIT;
		if (b < 0) b = 0;
		if (b > C_LIMIT) b = C_LIMIT;
		pixelbuffer[ARRAY(i, j)] = { r, g, b, a, z };
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