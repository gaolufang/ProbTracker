#ifndef _C2DOFKF_H
#define _C2DOFKF_H

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include <iostream>
#include <stdio.h>
#include <math.h>

using namespace cv;
using namespace std;

class C2DOFKF
{
public:
	void init( Point pos, float theta );
	void update();
	void output_pre( Point & pos, float & theta );
	void output_correct( Point & pos, float & theta);
	void measure( Point pos, float theta );

	C2DOFKF();
	~C2DOFKF();

public:
	Point m_ptPrePt;
	float m_fPreTheta;
	Point m_ptCorPt;
	float m_fCorTheta;

private:
	CvKalman * m_cvKalman;
	CvMat * m_matState;
	CvMat * m_matProcess_noise;
	CvMat * m_matMeasurement;
	CvRandState m_cvRng;
};

#endif
