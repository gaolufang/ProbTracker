#ifndef CTRACKER_H
#define CTRACKER_H

#include <iostream>
#include <string>
#include <stdio.h>
#include <vector>
#include <math.h>
#include <fstream>

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "LGlbLib.h"
#include "C2DOFKF.h"
#include "LGeometric.h"
#include "TTracker.h"

#define FEATURE_AIA		0

#define TRACK_EKF		0
#define TRACK_KF		1
#define TRACK_MEANSHIFT 2
#define TRACK_PARTICLE  3

#define CTRACKER_TRAJ_FRAME 4

using namespace cv;
using namespace std;
using namespace GeoLib;

// This type is set up for store matching results between features list
// and the trackers list.
typedef struct MATCHRESULT
{
	public:
		// The id indicate the position of the matched tracker in the tracker list
		long tracker_id;
		// The id indicates the position of the matched feature in the feature list.
		long feature_id;
		// The match result, cornerness difference.
		float cornerDiff;
		// The match result, overlap area
		float overlap;
}_MATCHRESUL_;


class CTracker
{
	public:
		/* The accumulator for id for each tracker */
		long m_lIdCnt;
		/* Frame counter */
		long m_lFrameCnt;
		/* The number of Initial trackers for each key frame */
		long m_lInitTrackerNum;
		/* The number of trackers in current frame */
		long m_lCurrTrackerNum;
		/* Setting up a threshold for tracker lost rate */
		int  m_nLostThreshold;
		/* Scale factor for x-y coordination */
		Point2f m_ptScaleFactor;
		/* global scale factor */
		float m_fGlobalScale;

		/* Imported feature list */
		vector<OxfordFeature>* m_pAIAFeatureList;
		/* Tracker list */
		vector<TRACKER>*	m_pTrackerList;

		void AIAFeatureInput(  vector<OxfordFeature>* pFeatureList );
		void EKF_update( const Mat & img );
		void TrajAnalysis();
		void draw( Mat& img, const Scalar color1, const Scalar color2, const Scalar color3 );
		void init();
		void findById( int nId, Mat img );
		void write_file( const string strFileName );
		void write_txt_file( const string strFilename );

		CTracker();
		~CTracker();

	private:
		bool isAuxiliaryPoint( const vector<Point> traj1, const vector<Point> traj2 );
		void convPointToMat( const vector<Point> pts, Mat & mat );
		void clean();
		void reInitTracker( TRACKER * pTracker );
		float compareFeatureToTracker( TRACKER * pTracker, OxfordFeature * pFeature, MATCHRESULT &reslut );
		int updateBestMatch( TRACKER * pTracker, vector<MATCHRESULT> vMatchList );
		void verifyTrackerList();
		void verifyTracker( TRACKER * pTracker );

		/* Current frame image */
		Mat m_matCurrentImg;
		/* Previous frame image  */
		Mat m_matPreviousImg;
		/* Feature type definition */
		int m_nFeatureType;
		/* Tracker type definition */
		int m_nTrackerType;
		/* Image size */
		Size m_cvSize;
};

#endif
