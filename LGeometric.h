#ifndef _GEOMETRIC_H
#define _GEOMETRIC_H

#include <iostream>
#include <math.h>
#include <stdio.h>

#include "opencv/highgui.h"
#include "opencv/cv.h"

using namespace cv;
using namespace std;

namespace GeoLib
{
const float g_fPi = 3.1415926;

// Calculate the area of a ellipse
float GetEllipseArea( float r1, float r2 );

// OpenCV version, ellipse overlap function lists
// Get the outline rectangle of the ellipse, 
// and the minimal sub-picture of the image contains two ellipse
// Judge is the rectangles interaction
// If interaction, finding the overlap pixels
int TwoEllipsesOverlapArea( Point pt1, Size axes1, float fTheta1,
							Point pt2, Size axes2, float fTheta2 ); 
bool isTwoRectsIntersection( Rect rt1, Rect rt2 );
bool isPointInRect( Point pt, Rect rt );
bool isPointInEllipse( Point pt, Point ptCentre, Size axes, float fTheta );
float TwoEllipsesUnionArea( Point pt1, Size axes1, float fTheta1,
							Point pt2, Size axes2, float fTheta2, float & overlap );

Rect getEllipseTanRect( Point pt, Size axes, float fTheta );
}// namespace
#endif
