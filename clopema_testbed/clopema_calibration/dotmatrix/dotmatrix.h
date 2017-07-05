
//	Library header

//	DotMatrix dot detection library
//	Radek Svoboda, November 2012, Neovision Ltd, Prague

/* ------------------------------------------------------------------------- */

#pragma once

/* ------------------------------------------------------------------------- */

#ifdef	_WIN32
#define	DLL_EXPORTED	extern "C" __declspec(dllexport)
#else
#define	DLL_EXPORTED	extern "C" __attribute__ ((visibility ("default")))
#endif

/* ------------------------------------------------------------------------- */

// library error codes

enum DMError

{
	DMERROR_OK,						// everything worked OK
	DMERROR_INVALID_PARAMETER,		// function parameter is invalid
	DMERROR_UNKNOWN_ALGORITHM,		// algorithm is unknown
	DMERROR_INVALID_SYNTAX,			// invalid algorithm parameters syntax
	DMERROR_KEY_NOT_FOUND,			// key dots were not found 
	DMERROR_CANT_READ_FILE,			// problem reading given file
	DMERROR_INVALID_FILE,			// invalid format of file
	DMERROR_INTERNAL				// other internal error
};

/* ------------------------------------------------------------------------- */

// image reference to existing memory, pixels stored row-by-row, top-to-bottom

struct DMImageRef

{
	int				width;			// image width (in pixels)
	int				height;			// image height (in pixels)
	int				memW;			// memory width to next row (in bytes)
	void *			data;			// start of image pixels
};

/* ------------------------------------------------------------------------- */

// single detected dot information

struct DMDot

{
	double			imgRow;			// image row position (subpixel)
	double			imgCol;			// image col position (subpixel)
	double			imgArea;		// image area of dot in pixels

	int				posRow;			// matrix grid position row
	int				posCol;			// matrix grid position col
};

/* ------------------------------------------------------------------------- */

// handle to created detector

typedef void * DMDetector;

// handle to dot matrix gauge

typedef void * DMGauge;

/* ------------------------------------------------------------------------- */

// prepare new detector

DLL_EXPORTED
DMDetector DMCreateDetector();

/* ------------------------------------------------------------------------- */

// destroy existing detector

DLL_EXPORTED 
void DMDestroyDetector(DMDetector detector);

/* ------------------------------------------------------------------------- */

// add image algorithm to the detector processing pipeline

DLL_EXPORTED
DMError DMAddImageAlgorithm(DMDetector detector, const char * algorithm);

/* ------------------------------------------------------------------------- */

// add shape algorithm to the detector processing pipeline

DLL_EXPORTED
DMError DMAddShapeAlgorithm(DMDetector detector, const char * algorithm);

/* ------------------------------------------------------------------------- */

// add detection algorithm to the detector processing pipeline

DLL_EXPORTED
DMError DMAddDetectionAlgorithm(DMDetector detector, const char * algorithm);

/* ------------------------------------------------------------------------- */

// process given image through the pipeline

DLL_EXPORTED
DMError DMProcessImage(DMDetector detector, const DMImageRef * image);

/* ------------------------------------------------------------------------- */

// get number of detected dots and pointer to the array of them

DLL_EXPORTED
DMError DMGetDetectedDots(DMDetector detector, int * numDots, 
						  const DMDot * * dots);

/* ------------------------------------------------------------------------- */

// prepare new dot matrix gauge

DLL_EXPORTED
DMGauge DMCreateGauge();

/* ------------------------------------------------------------------------- */

// destroy existing dot matrix gauge

DLL_EXPORTED 
void DMDestroyGauge(DMGauge gauge);

/* ------------------------------------------------------------------------- */

// add dot with radius to the dot matrix gauge

DLL_EXPORTED 
DMError DMAddDotToGauge(DMGauge gauge, double x, double y, double radius);

/* ------------------------------------------------------------------------- */

// load all dots from specified GDF file

DLL_EXPORTED
DMError DMLoadGdfFile(DMGauge gauge, const char * filename);

/* ------------------------------------------------------------------------- */

// arrange gauge dots, get number of dots and pointer to the array of them

DLL_EXPORTED
DMError DMGetArrangedDots(DMGauge gauge, int * numDots, const DMDot * * dots);

/* ------------------------------------------------------------------------- */

