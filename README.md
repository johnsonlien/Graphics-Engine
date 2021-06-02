# CSCI 580 - 3D Graphics 
This is a C++ Graphics Engine for a graphics course in USC. It uses Microsoft Visual Studios 2017.


# Overview
Throughout the semester, I continuously build onto this project to create a working graphics library. A "skeleton" program was given to us and we had to fill in the missing function definitions. Each homework assignment focuses on different aspects of a graphics library and is summarized below. 


## Homework 1: Rendering
I work on the display module and write functions that render the images onto the screen.


## Homework 2: Z-Buffer
I implement an additional layer of depth to give a 3-dimensional feeling to the images using what is called a z-buffer. Z-buffer will tell the program which pixels will be displayed in front of the screen and which pixels will be in the back. Pixels with smaller z-values will be the ones to be displayed while other pixels with higher z-values will be behind.


## Homework 3: Transformations
This program focuses on performing transformations on a model. Any sort of sliding, scaling, or rotations will be calculated to produce an accurate-looking image. 


## Homework 4: Shaders
I add shaders to our library and implement three different shading techniques:
* flat shading
* Gouraud shading
* Phuong shading

### Flat Shading
Flat shading is where you choosing a vertex's color and that will be the polygon's color
### Gouraud Shading
A form of smooth shading where you perform bilinear interpolation between the three points of the triangle to calculate what color the polygon will be.
### Phuong Shading
Similar to Gouraud shading but this time we will be interpolating the normals of each pixel of each triangle.


## Homework 5: Textures
This enables textures to be applied onto our models by taking in any ppm file.


## Homework 6: Antialiasing
To improve image quality, we perform antialiasing to remove jagged edges from the image. 
