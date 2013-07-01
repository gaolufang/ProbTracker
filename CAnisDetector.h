#ifndef _CLASS_ANISDETECTOR_H
#define _CLASS_ANISDETECTOR_H

#include <iostream>
#include <fstream>

#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv/cxcore.h"

#include "COxfordFeatureList.h"
#include "LGlbLib.h"

using namespace std;
using namespace cv;

class CAnisDetector:public COxfordFeatureList
{
	/*  Functions List */
public:
	void update( const Mat & img );	

	CAnisDetector();
	~CAnisDetector();
};

#endif
