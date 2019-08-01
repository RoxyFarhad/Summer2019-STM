//render ultrasound data
//based on code by Saveliy Yusufov <sy2685@columbia.edu>

#include <cstdio>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <cstring>

#include <math.h>

//#include <pthread.h>
#include <vector>
#include <mutex>

//#include "server.cpp"
//#include "safe_queue.h"

#include "SDL.h"

#include "ultrasound_rendering.h"


//define constants for the rendering window
#define SCREEN_WIDTH 700
#define SCREEN_HEIGHT 700
#define TEX_HEIGHT 700
#define TEX_WIDTH 700

//define constants for encoder resolution and data points per tx
#define ENCODER_RES 4096
#define NUM_DATA_POINTS 500
#define MARKER_LEN 10
#define OVERHEAD_LEN 12

//define constant for length of rays
#define RAY_LEN 500


//error print message and quit function
void error(const char* errorMessage) {
	std::cerr << errorMessage << std::endl;
	exit(1);
}

//helper function for converting bytes to encoder value
static inline Uint16 bytesToEncVal(Uint8 byte1, Uint8 byte2) {
	return (((Uint16)byte1) << 8) | byte2;			//might not need the cast
}

//helper function for converting encoder val to angle w.r.t initial angle
//---------------this is poorly written----------------------------//
//----probably could use some sort of ternary---------//
static inline double encValToAngle(Uint16 curVal) {
	return (((double) curVal) * 2.0 * M_PI / ENCODER_RES) + (3.0 * M_PI / 2.0);
}

/*helper function for renderer: takes in reference to bool and checks if the 
	renderer is still running. If not, it sets the bool to false
*/
void process_events(SDL_bool& running) {
	SDL_Event event;

	while (SDL_PollEvent(&event)) {
		if (event.type == SDL_QUIT) {
			running = SDL_FALSE;
		}
	}
}

//helper functions for converting values to points on the screen
static inline SDL_Point polar_to_cart(double r, double theta) {
	SDL_Point point;
	point.x = (int)(r * cos(theta));
	point.y = (int)(r * sin(theta));
	return point;
}

static inline SDL_Point cart_to_screen(SDL_Point cart_pt) {
	cart_pt.x += (int)(SCREEN_WIDTH * 0.5);
	cart_pt.y = (int)((-1) * cart_pt.y);
	return cart_pt;
}

//helper function to duplicate byte 4 times into int
static inline int colorAlphaToInt(Uint8 color, Uint8 alpha) {
	return (alpha << 24) + (color << 16) + (color << 8) + color;
}


/*
 * Draw a line from the start point, `p1` to the end point, `p2`
 * Renders the line by drawing pixels from `p1` to `p2`
 */
static inline void draw_line(SDL_Point p1, SDL_Point p2, tx_interval& tx_itval,
	Uint32 *pix) {
	//copy over information
	int x0 = p1.x;
	int y0 = p1.y;
	int x1 = p2.x;
	int y1 = p2.y;
	
	//placeholder vars for current pixel
	int xi;
	int yi;

	//vars for tracking color
	int color;
	int alpha;
	Uint8 value;

	//for each tx intensity value
	for (int i = 0; i < NUM_DATA_POINTS; i++) {
		//load the next intensity value
		value = tx_itval.intensities[OVERHEAD_LEN + i];

		//calculate the current xi yi
		xi = x0 + int((x1 - x0) * i / NUM_DATA_POINTS);
		yi = y0 + int((y1 - y0) * i / NUM_DATA_POINTS);


		//calculate the pixel position in the array
		unsigned int offset = TEX_WIDTH * yi + xi;

		// For LOW voltage, the pixel is black and transparent
		// For HIGH voltage, the pixel is white and opaque
		// Otherwise, the pixel's color & transparency is the voltage value
		if (value <= 27) {
			color = 0;
			alpha = SDL_ALPHA_TRANSPARENT;
		}
		else if (value > 199) {
			color = 255;
			alpha = SDL_ALPHA_OPAQUE;
		}
		

		else {
			color = value;
			alpha = value;
		}

		//update pixel array
		pix[offset] = colorAlphaToInt(color, alpha);
	}
}

//redraws the screen
void draw_screen(SDL_Renderer* renderer, SDL_Texture* texture, 
				 Uint32 *pixels) 
{
	//update the texture
	SDL_UpdateTexture(texture, NULL, pixels, TEX_WIDTH * sizeof(Uint32));

	//render
	SDL_RenderClear(renderer);
	SDL_RenderCopy(renderer, texture, NULL, NULL);
	SDL_RenderPresent(renderer);
}


int main(int argc, char** argv)
{
	//-------------------file reading stuff------------------------//
	//read in binary file looking for the first marker
	//vars for the file objects and file names
	FILE *fileIn;

	//file name. one is scans on is test
	const char *fileName = "C:\\Users\\fumin\\Desktop\\7_25_scans\\7_25_scan5.txt";
	//const char *fileName = "C:\\Users\\fumin\\Desktop\\test.txt";
	errno_t fileError;

	//vars for marker
	char marker[] = "\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff";
	bool markerFound = false;
	char* markerStart;

	//vars for tracking reading and the input buffer
	size_t numRead = 0;
	long offset = 0;
	char buf[ARRAY_LEN + 1];

	//null terminate the buffer for ease of computation
	buf[ARRAY_LEN] = '\0';

	//open the file
	fileError = fopen_s(&fileIn, fileName, "rb");
	if (fileError != 0) {
		error("failed fopen of input");
	}

	//find first set of 10 0xff (marker)
	while (!markerFound) {
		//read in bufLen bytes
		numRead = fread(buf, 1, ARRAY_LEN, fileIn);

		if (numRead != ARRAY_LEN) {
			//exit
			std::cerr << "failed fread at start" << std::endl;
			std::cerr << "numRead: " << numRead << std::endl;
			return 1;
		}

		//check for marker in the read string
		markerStart = strstr(buf, marker);

		//if marker found, set markerFound, fseek to start of marker
		if (markerStart) {
			markerFound = true;

			//calculate offset, move pointer back
			offset = -1 * strlen(markerStart);
			fseek(fileIn, offset, SEEK_CUR);
		}
	}


	
	

	//-----------------------------rendering stuff -------------------//


	//vars to keep track of rendering status
	SDL_bool running = SDL_TRUE;
	
	//vars for pixels in the rendering screen and tx_interval
	Uint32 *pixels = new Uint32[TEX_WIDTH * TEX_HEIGHT];
	tx_interval data;

	//black out the pixel array
	for (int i = 0; i < TEX_WIDTH * TEX_HEIGHT; i++) {
		pixels[i] = 255 << 24;
	}


	//null terminate the array
	data.intensities[ARRAY_LEN] = '\0';

	//vars for starting angle
	double startAngle;
	const double ANGLE_STRAIGHT_DOWN = 3 * M_PI / 2;

	//var for encoder position
	Uint16 encoderPos;

	
	//read first set of data directly into tx_interval
	numRead = fread(data.intensities, 1, ARRAY_LEN, fileIn);
	if (numRead != ARRAY_LEN) {
		error("fread error");
	}
	

	/*
	//super fake data
	for (int i = 0; i < ARRAY_LEN; i++) {
		data.intensities[i] = i % 255;
	}
	*/

	//set initial encoder value, starting angle, initial vals
	encoderPos = bytesToEncVal(data.intensities[MARKER_LEN], data.intensities[MARKER_LEN + 1]);
	startAngle = encValToAngle(encoderPos);
	data.angle = ANGLE_STRAIGHT_DOWN;

	//vars for setting screen origin as an SDL_Point and for end point
	const int X_ORIGIN = SCREEN_WIDTH / 2;
	const int Y_ORIGIN = 0;
	SDL_Point start_pt;
	SDL_Point end_pt;
	start_pt.x = X_ORIGIN;
	start_pt.y = Y_ORIGIN;



	//initialize SDL
	if (SDL_Init(SDL_INIT_EVERYTHING) != 0) {
		std::cerr << "SDL_Init failed: " << SDL_GetError() << std::endl;
		return 1;
	}

	//create SDL window
	SDL_Window* window = SDL_CreateWindow("Columbia Open-Source UltraSound",
		SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
		SCREEN_WIDTH, SCREEN_HEIGHT, 0);

	//create SDL renderer
	SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, 0);

	//creeate SDL texture
	SDL_Texture* texture = SDL_CreateTexture(renderer,
		SDL_PIXELFORMAT_ARGB8888,
		SDL_TEXTUREACCESS_STATIC,
		TEX_WIDTH,
		TEX_HEIGHT);

	//set rendering color to black
	SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
	SDL_UpdateTexture(texture, NULL, pixels, TEX_WIDTH * sizeof(Uint32));

	//reset renderer and prepare for rendering
	SDL_RenderClear(renderer);
	SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
	SDL_RenderPresent(renderer);


	

	//main rendering loop
	while (running) {
		//check if renderer was closed using helper function
		process_events(running);

		
		//test: play with angles
		if (data.angle < 5.00) {
			data.angle += 0.0001;
		}
		else {
			data.angle = ANGLE_STRAIGHT_DOWN;
		}
		

		//calculate end point
		end_pt = polar_to_cart(RAY_LEN, data.angle);
		end_pt = cart_to_screen(end_pt);

		//draw the line and update the screen
		draw_line(start_pt, end_pt, data, pixels);
		draw_screen(renderer, texture, pixels);
		
		//read in next set of data
		fread(data.intensities, 1, ARRAY_LEN, fileIn);

		//check for marker
		if (strstr(data.intensities, marker) != data.intensities) {
			error("marker not found");
		}

		//update angle
		//calculate encoder pos and corresponding angle
		encoderPos = bytesToEncVal(data.intensities[MARKER_LEN], data.intensities[MARKER_LEN + 1]);
		data.angle = encValToAngle(encoderPos) - startAngle + ANGLE_STRAIGHT_DOWN;
	}















	//once renderer is closed, deallocate resources and quit SDL
	if (texture) {
		SDL_DestroyTexture(texture);
	}
	if (renderer) {
		SDL_DestroyRenderer(renderer);
	}
	if (window) {
		SDL_DestroyWindow(window);
	}
	SDL_Quit();
	delete[] pixels;
	fclose(fileIn);


	return 0;
}