#ifndef PROB_TRACKER_CXX
#define PROB_TRACKER_CXX

#include <iostream>
#include <stdio.h>
#include <string>

#include "opencv2/highgui/highgui.hpp"
#include "opencv/cv.h"

#include "LGlbLib.h"
#include "CSource.h"
#include "CTracker.h"
#include "CAnisDetector.h"
#include "CStopwatch.h"
#include "COpenPrj.h"

#define PRJ_ROTATION "/home/lufang/Research/Output/Rotation/Rotation.prj"
#define PRJ_PAN		 "/home/lufang/Research/Output/Pan/Pan.prj"
#define PRJ_ZOOM	 "/home/lufang/Research/Output/Zoom/Zoom.prj"
#define PRJ_MOSER_FU "/home/lufang/Research/Output/Moser_Fu/MoserFu.prj"
#define CURRENT_PRJ  2

#define VALIDATION 1

using namespace cv;
using namespace std;
using namespace GLBLIB;

int main( int argc, const char** argv )
{
	/* Setup Current Project */
	string strProjectFileName;
	switch( CURRENT_PRJ )
	{
	case 0:
		strProjectFileName = Char2String( PRJ_ROTATION );
		break;
	case 1:
		strProjectFileName = Char2String( PRJ_PAN );
		break;
	case 2:
		strProjectFileName = Char2String( PRJ_ZOOM );
		break;
	case 3:
		strProjectFileName = Char2String( PRJ_MOSER_FU );
		break;
	default:
		cout<<"Invalid project filename."<<endl;
		return 0;
	}

	/* Setup project & dataset */
	COpenPrj pOpenProject( strProjectFileName ); 
	
	/* Parameter configuration */
	/*  - Feature selection parameters */
	int 	nHalf_window_size = 20;
	float 	fFeature_selection_thresh = 0.4;
	int 	nMaxFeatureNum = 150;

	/* Initialize detector, tracker and image source */
	CAnisDetector * pAnisList = new CAnisDetector;
	CTracker 	  * pTracker = new CTracker;
	CSource 	  * pSource = new CSource; 
	
	pAnisList->SetFilePrefix( pOpenProject.FeaturePrefix(), pOpenProject.FeatureType(), pOpenProject.FeatureBegin(), pOpenProject.FeatureEnd() );
	pSource->SetImageFilePrefix( pOpenProject.ImgPrefix(), pOpenProject.ImgType(), pOpenProject.ImgBegin(), pOpenProject.ImgEnd(), pOpenProject.ImgBit() );
	pSource->init();

	/* Define the color for drawing features & trackers */
	Scalar sGreen( 0, 255, 0);
	Scalar sBlue( 255, 0, 0 );
	Scalar sBlack( 0, 0, 0 ); 	
	Scalar sWhite( 255, 255, 255 ); 	
	Scalar sRed( 0, 0, 255 ); 	
	Scalar sYellow( 0, 255, 255 ); 	
	
	/* Main loop */
	Mat img;
	while ( true )
	{
		/* Update the video sequences source  */
		img = pSource->update();  
		/* Check the video source is available */
		if( pSource->isOpened() )
		{
			/* Setup the paths for th output files */
			string strNum = Int2String( pSource->m_lFrameNum - 1 );
			string strOutFullFileName = pOpenProject.OutputFilePrefix() + strNum + ".txt";
			string strOutFullImageName = pOpenProject.OutputImgPrefix() + strNum + ".jpg";
		
			/* Calculate the consumer of the computational time */
			CStopwatch timer(true);
			
			/* Feature detector update */
			pAnisList->update( img );	//Feature detection or load from file
			pAnisList->filter( img, nHalf_window_size, fFeature_selection_thresh, nMaxFeatureNum );
	
			/* Tracker update */
			pTracker->AIAFeatureInput( pAnisList->OutputList() );
			pTracker->EKF_update( img );

			/* Tracking by specific ID */
			//pTracker->findById( 164, img );

			/* Draw the feature or trackers on the image */
			pTracker->draw( img, sBlue, sGreen, sBlack );
			pAnisList->draw( img, sYellow );

			cout<<"Frame: "<<pSource->FrameNum()<<endl;
			imshow("img", img);

			/* Output the result to the images and xml files */
			imwrite( strOutFullImageName, img );
			pTracker->write_txt_file( strOutFullFileName );
			
			/* Calculate the processing time */
			cout<<"Processing rate: "<<1.0/timer.Elapsed()<<"fps."<<endl;

			int key = waitKey( 3 );
			switch( key )
			{
				/* Press space for stop */
				case 32:
					waitKey( 0 );
					break;
				/* Press ESC for quit */
				case 27:

					return 0;
					break;

				default:
					continue;
					break;
			}
		}
		else
		{
			break;
		}
	}
}
#endif
