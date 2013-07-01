#include "LGeometric.h"
namespace GeoLib
{
float GetEllipseArea( float r1, float r2 )
{
	float s = g_fPi * r1 * r2;
	return s;
}

int TwoEllipsesOverlapArea( Point pt1, Size axes1, float theta1, 
	                        Point pt2, Size axes2, float theta2 )
{
	// Get the outline rectangle from the 1st ellipse
	Rect rt1 = getEllipseTanRect( pt1, axes1, theta1 );
	//rt1.height = ( axes1.width > axes1.height ) ? axes1.width : axes1.height;
	//rt1.width = rt1.height;
	
	// Get the outline rectangle from the 2nd ellipse
	Rect rt2 = getEllipseTanRect( pt2, axes2, theta2 );
	
	// Check the interaction between the two rectangles
	if( !isTwoRectsIntersection( rt1, rt2 ) )
	{
		return 0;
	}
	
	// Find the small one in two Rect
	Rect rtSeach;
	if( rt1.width * rt1.height > rt2.width * rt2.height )
	{
		rtSeach = rt2;
	}
	else
	{
		rtSeach = rt1;
	}	
	
	// Search in the-subwindow of the img			
	//Mat img;
	//img = Mat::zeros(400, 400, CV_8U);
	
	int nOverlapArea = 0;
	for(int i = rtSeach.x; i < rtSeach.x + rtSeach.width; i ++ )
	{
		for( int j = rtSeach.y; j < rtSeach.y + rtSeach.height; j ++ )
		{
			Point pt;
			pt.x = i;
			pt.y = j;
			if(    isPointInEllipse(pt, pt1, axes1, theta1) 
				&& isPointInEllipse(pt, pt2, axes2, theta2) )
			{
				//img.at<uchar>(j, i) = 255;
				nOverlapArea ++;
			}
		}
	}

	//ellipse( img, pt1, axes1, theta1, 0, 360, Scalar(255) );
	//ellipse( img, pt2, axes2, theta2, 0, 360, Scalar(255) );
	//imshow("img", img);
	//waitKey(0);
	return nOverlapArea;
}

bool isTwoRectsIntersection( Rect rt1, Rect rt2 )
{
	// Four corner points for 1st rectangle
	Point pt1[4]; 
	Point pt2[4]; 
	
	pt1[0].x 	= rt1.x; 				pt1[0].y 	= rt1.y;
	pt1[1].x 	= rt1.x + rt1.width; 	pt1[1].y 	= rt1.y;
	pt1[2].x 	= rt1.x; 				pt1[2].y 	= rt1.y + rt1.height;
	pt1[3].x	= rt1.x + rt1.width;	pt1[3].y	= rt1.y + rt1.height;

	for( int i = 0; i < 4; i ++ )
	{
		if( isPointInRect( pt1[i], rt2 ) )
		{
			return true;
		}
	}
	
	pt2[0].x 	= rt2.x; 				pt2[0].y 	= rt2.y;
	pt2[1].x 	= rt2.x + rt2.width; 	pt2[1].y 	= rt2.y;
	pt2[2].x 	= rt2.x; 				pt2[2].y 	= rt2.y + rt2.height;
	pt2[3].x	= rt2.x + rt2.width;	pt2[3].y	= rt2.y + rt2.height;

	for( int i = 0; i < 4; i ++ )
	{
		if( isPointInRect( pt2[i], rt1 ) )
		{
			return true;
		}
	}

	return false;
}

bool isPointInRect( Point pt, Rect rt )
{
	bool bResult =  (pt.x >= rt.x) && (pt.x <= rt.x + rt.width);
 
	return( (pt.x >= rt.x) && (pt.x <= rt.x + rt.width)
			&& (pt.y >= rt.y) && ( pt.y <= (rt.y + rt.height) ) ); 
}

bool isPointInEllipse( Point pt, Point ptCentre, Size axes, float fTheta )
{
	float cos_theta = cos( fTheta * g_fPi / 180 );
	float sin_theta = sin( fTheta * g_fPi / 180 );
	float func = 	pow( (pt.x-ptCentre.x)  * cos_theta + (pt.y-ptCentre.y)  * sin_theta, 2) 
				/	pow( axes.width, 2 )
				+	pow( (pt.x-ptCentre.x)  * sin_theta - (pt.y-ptCentre.y)  * cos_theta, 2)
				/	pow( axes.height, 2 ); 
	if( func > 1 )
	{
		return false;
	}
	else
	{
		return true;
	}
}

float TwoEllipsesUnionArea( Point pt1, Size axes1, float fTheta1,
							Point pt2, Size axes2, float fTheta2, float & overlap )
{
	float fArea1 = GetEllipseArea( axes1.width, axes1.height );
	float fArea2 = GetEllipseArea( axes2.width, axes2.height );
	float fAreaTotal = fArea1 + fArea2;
	float fOverlapArea = TwoEllipsesOverlapArea( pt1, axes1, fTheta1, pt2, axes2, fTheta2);
	
	overlap = fOverlapArea;
	return ( fAreaTotal - fOverlapArea );
}

Rect getEllipseTanRect( Point pt, Size axes, float fTheta )
{
	float cos_theta = pow( cos( fTheta * g_fPi / 180 ), 2 );
	float sin_theta = pow( sin( fTheta * g_fPi / 180 ), 2 );
	float r1_2 = pow( axes.width, 2 );
	float r2_2 = pow( axes.height, 2 );

	float xLimit = sqrt( r1_2 * cos_theta + r2_2 * sin_theta );
	float yLimit = sqrt( r1_2 * sin_theta + r2_2 * cos_theta );

	Rect rt;
	rt.width = round( 2 * xLimit );
	rt.height = round( 2 * yLimit );
	rt.x = round( pt.x - xLimit );
	rt.y = round( pt.y - yLimit );

	return rt;
}

}// namespace
