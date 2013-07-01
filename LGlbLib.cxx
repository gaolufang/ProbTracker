
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "opencv/highgui.h"
#include "opencv/cv.h"
#include "LGlbLib.h"

using namespace std;
using namespace cv;

string GLBLIB::Int2String ( int number )
{
	std::stringstream srmMedian;
	string strResult;
	
	srmMedian << number;
	srmMedian >> strResult;

	return strResult;
}

string GLBLIB::Int2BitString( int number, int bit )
{
	std::stringstream srmMedian;
	string strResult;

	srmMedian << number;
	srmMedian >> strResult;

	if ( strResult.length() < bit )
	{
		int diff = bit - strResult.length();
		for( int i = 0; i < diff; i ++ )
		{
			strResult = "0" + strResult;
		}
	}

	return strResult;
}

string GLBLIB::Char2String( const char * strChar )
{
	string strResult(strChar);
	return strResult;
}

void GLBLIB::nonmaxsupres_opencv( 	const Mat& img, Mat& dist, vector<Point>& pts, 
							float thresh = 1, int window_half_size = 4 )
{
	int nWSize = 2 * window_half_size + 1;

	switch( img.depth() )
	{
		case CV_16S:
			dist = Mat::zeros( img.rows, img.cols, CV_16S );
			for( int i = 0; i < img.rows; i ++ )
			{
				const int* img_data = img.ptr<int>(i);
				int* 		 filter_data = dist.ptr<int>(i);
				for ( int j = 0; j < img.cols; j ++ )
				{
					// Find the maximum value in the region
					// if the pixel is maximum, keep it, otherwise replace by 0
					
					if( img_data[j] < thresh )
					{
						filter_data[j] == 0;
						continue;
					}

					int nRStart = ( (i-window_half_size) >= 0 ) ?
								  ( i - window_half_size )		:
								  0;
					int nREnd 	= ( (i+window_half_size) < img.rows ) ?
								  ( i + window_half_size )			  :
								  img.rows - 1;
					int nCStart = ( (j-window_half_size) >= 0 ) ?
								  ( j - window_half_size )		:
								  0;
					int nCEnd 	= ( (j+window_half_size) < img.cols ) ?
								  ( j + window_half_size )			  :
								  img.cols - 1;
					float fMax = 0;
					for( int m = nRStart; m <= nREnd; m ++ )
					{
						const int* data = img.ptr<int>(m);
						for( int n = nCStart; n <= nCEnd; n ++ )
						{
							
							if( data[n] >= fMax )
							{
								fMax = data[n];
							}
						}
					}
					 
					if ( img_data[j] == fMax )
					{
						filter_data[j] = fMax;
					}
					else
					{
						filter_data[j] = 0.0;
					}
				}
			}
			break;

		case CV_32F:
			dist = Mat::zeros( img.rows, img.cols, CV_32F );
			for( int i = 0; i < img.rows; i ++ )
			{
				const float* img_data = img.ptr<float>(i);
				float* 		 filter_data = dist.ptr<float>(i);
				for ( int j = 0; j < img.cols; j ++ )
				{
					// Find the maximum value in the region
					// if the pixel is maximum, keep it, otherwise replace by 0
					
					if( img_data[j] < thresh )
					{
						filter_data[j] == 0;
						continue;
					}

					int nRStart = ( (i-window_half_size) >= 0 ) ?
								  ( i - window_half_size )		:
								  0;
					int nREnd 	= ( (i+window_half_size) < img.rows ) ?
								  ( i + window_half_size )			  :
								  img.rows - 1;
					int nCStart = ( (j-window_half_size) >= 0 ) ?
								  ( j - window_half_size )		:
								  0;
					int nCEnd 	= ( (j+window_half_size) < img.cols ) ?
								  ( j + window_half_size )			  :
								  img.cols - 1;
					float fMax = 0;
					for( int m = nRStart; m <= nREnd; m ++ )
					{
						const float* data = img.ptr<float>(m);
						for( int n = nCStart; n <= nCEnd; n ++ )
						{
							
							if( data[n] >= fMax )
							{
								fMax = data[n];
							}
						}
					}
					 
					if ( img_data[j] == fMax )
					{
						filter_data[j] = fMax;
						Point pt;
						pt.x = j;
						pt.y = i;
						pts.push_back(pt);
					}
					else
					{
						filter_data[j] = 0.0;
					}
				}
			}
			break;
			
		defaut:
			;
	}	
}

float GLBLIB::EllipseOverlap( const Size img_size, Point pt1, Point pt2, 
					Size axes1, Size axes2, float theta1, float theta2 )
{
	Mat img1 = Mat::zeros( img_size.height, img_size.width, CV_16S );
	Mat img2 = Mat::zeros( img_size.height, img_size.width, CV_16S );

	Scalar color;
	color.val[0] = 255;
	
	ellipse( img1, pt1, axes1, theta1, 0, 360, color );
	ellipse( img2, pt2, axes2, theta2, 0, 360, color );
	
	Mat diff;
	absdiff( img1, img2, diff );
	Mat unio;
	add( img1, img2, unio );

	int nUnion = countNonZero( unio );
	int nDiff = nUnion - countNonZero( diff );

	return float(nDiff) / float(nUnion);
}


void GLBLIB::calcHistInEllipseRGB( Mat img, Mat & b_hist, Mat & g_hist, Mat & r_hist,
						   Point pt, Size axes, float fTheta )
{
	// split the image 
	vector<Mat> planes;
	split( img, planes );

	b_hist.create( 1, 256, CV_32F );
	g_hist.create( 1, 256, CV_32F );
	r_hist.create( 1, 256, CV_32F );
	
	b_hist = Scalar(0);
	g_hist = Scalar(0);
	r_hist = Scalar(0);
	
	// Get the out line of the ellipse
	Rect rt;
	rt.height = ( axes.width > axes.height ) ? axes.width * 2 : axes.height * 2;
	rt.width = rt.height;
	
	rt.x = ( pt.x - rt.width > 0 ) ? (pt.x - rt.width) : 0;
	rt.y = ( pt.y - rt.height > 0 ) ? (pt.y - rt.height) : 0;
	
	int col_search = ( rt.x + rt.width >= img.cols ) ? ( img.cols - 1 ) : ( rt.x +rt.width );
	int row_search = ( rt.y + rt.height >= img.rows ) ? ( img.rows - 1 ) : ( rt.y +rt.height );
	int n = 0;
	
	for( int i = rt.y; i < row_search; i ++ )
	{
		uchar* pB = planes[0].ptr<uchar>(i);
		uchar* pG = planes[1].ptr<uchar>(i);
		uchar* pR = planes[2].ptr<uchar>(i);

		for( int j = rt.x; j < col_search; j ++ )
		{
			Point pt_search;
			pt_search.x = j;
			pt_search.y = i;
			if( isPointInEllipse( pt_search, pt, axes, fTheta) )
			{
				b_hist.at<float>( pB[j] ) += 1.f;
				g_hist.at<float>( pG[j] ) += 1.f;
				r_hist.at<float>( pR[j] ) += 1.f;

				n ++;
			}
		}
	}

	// Normalize
	for( int i = 0; i < 256; i ++ )
	{
		b_hist.at<float>(i) /= n;
		g_hist.at<float>(i) /= n;
		r_hist.at<float>(i) /= n;
	}
}

