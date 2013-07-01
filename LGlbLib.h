#ifndef LGLBLIB_H
#define LGLBLIB_H

#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "opencv/highgui.h"
#include "opencv/cv.h"
#include "LGeometric.h"

using namespace std;
using namespace cv;
using namespace GeoLib;

typedef struct AffRegFeature 
{
	public:
		Point  pt;
		Size   axes;
		float theta;
		float A, B, C;
		float cornerness;
		
		AffRegFeature(void)
		{
			pt.x = 0; pt.y =0;
			axes.height = 0; axes.width = 0;
			theta = 0;
			A = 0; B = 0; C = 0;
			cornerness = 0;
		}
}_AffRegFeature_;

typedef struct OxfordFeature 
{
	public:
		Point  pt;
		Size   axes;
		float theta;
		float A, B, C;
		float cornerness;
		
		OxfordFeature(void)
		{
			pt.x = 0; pt.y =0;
			axes.height = 0; axes.width = 0;
			theta = 0;
			A = 0; B = 0; C = 0;
			cornerness = 0;
		}
}_OxfordFeature_;

namespace GLBLIB 
{
std::string Int2String ( int number );
std::string Int2BitString( int number, int bit );
std::string Char2String( const char * strChar );
void nonmaxsupres_opencv( const Mat& img, Mat& dist, vector<Point>& pts, float thresh, int window_half_size );
void nonmaxsupres( const Mat& img, Mat& dist, float thresh, int window_half_size );
float EllipseOverlap( const Size img_size, Point pt1, Point pt2, Size axes1, Size axes2, float theta1, float theta2 );
void calcHistInEllipseRGB( Mat img, MatND & b_hist, MatND & g_hist, MatND & r_hist,
						   Point pt, Size Axes, float fTheta );
}
#endif
