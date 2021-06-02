/* Texture functions for cs580 GzLib	*/
#include    "stdafx.h" 
#include	"stdio.h"
#include	"Gz.h"
#include	<complex>	// Imaginary & real values

GzColor	*image = NULL;
int xs, ys;
int reset = 1;

/* Image texture function */
int tex_fun(float u, float v, GzColor color)
{
	unsigned char		pixel[3];
	unsigned char     dummy;
	char  		foo[8];
	int   		i, j;
	FILE			*fd;

	if (reset) {          /* open and load texture file */
		fd = fopen("texture", "rb");
		if (fd == NULL) {
			fprintf(stderr, "texture file not found\n");
			exit(-1);
		}
		fscanf(fd, "%s %d %d %c", foo, &xs, &ys, &dummy);
		image = (GzColor*)malloc(sizeof(GzColor)*(xs + 1)*(ys + 1));
		if (image == NULL) {
			fprintf(stderr, "malloc for texture image failed\n");
			exit(-1);
		}

		for (i = 0; i < xs*ys; i++) {	/* create array of GzColor values */
			fread(pixel, sizeof(pixel), 1, fd);
			image[i][RED] = (float)((int)pixel[RED]) * (1.0 / 255.0);
			image[i][GREEN] = (float)((int)pixel[GREEN]) * (1.0 / 255.0);
			image[i][BLUE] = (float)((int)pixel[BLUE]) * (1.0 / 255.0);
		}

		reset = 0;          /* init is done */
		fclose(fd);
	}

	/* bounds-test u,v to make sure nothing will overflow image array bounds */
	/* determine texture cell corner values and perform bilinear interpolation */
	/* set color to interpolated GzColor value and return */

		/* Bound u and v */
	if (u > 1.0f) u = 1.0f;
	if (u < 0.0f) u = 0.0f;
	if (v > 1.0f) v = 1.0f;
	if (v < 0.0f) v = 0.0f;

	/* Scale u and v so now they are [0, xres-1] and [0, yres-1] */
	float scale_u = u * (xs - 1);
	float scale_v = v * (ys - 1);

	/* Get the bounding box */
	int maxX = (int)ceil(scale_u);
	int minX = (int)floor(scale_u);

	int maxY = (int)ceil(scale_v);
	int minY = (int)floor(scale_v);

	float s = scale_u - minX;
	float t = scale_v - minY;

	GzColor A, B, C, D;

	for (int rgb = 0; rgb < 3; rgb++) {
		A[rgb] = image[minY * xs + minX][rgb];
		B[rgb] = image[minY * xs + maxX][rgb];
		C[rgb] = image[maxY * xs + maxX][rgb];
		D[rgb] = image[maxY * xs + minX][rgb];
		color[rgb] = s * t * C[rgb] + (1.0f - s) * t * D[rgb] + s * (1.0f - t) * B[rgb] + (1.0f - s) * (1.0f - t) * A[rgb];
	}

	return GZ_SUCCESS;
}

/* Procedural texture function */
int ptex_fun(float u, float v, GzColor color)
{

	/* Bound u and v values */
	if (u > 1.0f) u = 1.0f;
	if (u < 0.0f) u = 0;
	if (v > 1.0f) v = 1.0f;
	if (v < 0.0f) v = 0;
	
	int N;
	N = 6;
	int scale_u = (int)round(u * N);
	int scale_v = (int)round(v * N);

	if (scale_u % 2 == scale_v % 2) { /* Same interval*/
		color[RED] = 0.0f;
		color[GREEN] = 0.0f;
		color[BLUE] = 0.1f;
	}
	else {
		color[RED] = 0.2f;
		color[GREEN] = 0.8f;
		color[BLUE] = 0.3f;
	}
	
	return GZ_SUCCESS;
}

/* Free texture memory */
int GzFreeTexture()
{
	if(image!=NULL)
		free(image);
	return GZ_SUCCESS;
}

