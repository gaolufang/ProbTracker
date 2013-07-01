#ifndef TTRACKER_H
#define TTRACKER_H

#include <iostream>
#include <string>
#include <stdio.h>
#include <vector>
#include <math.h>

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "LGlbLib.h"
#include "C2DOFKF.h"
#include "LGeometric.h"

#define TRACKER_TRAJ_FRAME 10

using namespace GLBLIB;

// Structure of tracker, including the verification, and recover from failure
typedef struct TRACKER
{
	public:
		// The numbers of consistent successful tracked
		int  nSucces;
		// The number of consistent failure tracked
		int  nFail;
		// The number of tracked
		int  nTracked;
		// The id of the tracker
		long id;
		// The index of frame last updated
		long last_update;
		// The matching result of the id in feature list
		long match_id;
		// No used
		bool isSalient;
		// Predicted position from EKF filter
		Point predict_pt;
		// Predicted Size from global factor 
		Size predicted_axes;
		// Predicted theta from EKF filter
		float predict_theta;
		// Estimated result of trajectory verification
		Point estimate_pt;
		Size  estimate_axes;
		float estimate_scale;
	
		// Previous tracker state
		Point prev_pt;
		Size  prev_axes;
		float prev_theta;

		// Matched tracker result;
		Point 	matched_pt;
		Size	matched_axes;
		float	matched_theta;

		// The feature including current tracker
		OxfordFeature* a_feature;
		// Kalman filter
		C2DOFKF EKF;
		// Trajectory of tracker
		vector<Point> * vTrj;
		vector<Size> vAxes;
		// RGB histogram
		MatND b_hist, g_hist, r_hist;
	
		TRACKER( void )
		{
			id = 0;
			match_id = -1;
			last_update = 0;
		}
		
		// Initial function of the TRACKER
		// img - image of current frame
		// nId - the id of the TRACKER
		// nLastUpdate - current frame no.
		// nMatchID - The match result of feature
		// featue - mathced feature
		TRACKER( const Mat img, int nId, int nLastUpdate, int nMatchID, OxfordFeature feature )
		{
			vTrj = new vector<Point>;
			Point pt = feature.pt;
			vTrj->push_back( Point(0, 0) );
			vAxes.push_back( feature.axes );

			isSalient = false;
			id = nId;
			last_update = nLastUpdate;
			nSucces = 1;
			nFail 	= 0;
			nTracked = 1;

			a_feature = new OxfordFeature;

			*a_feature = feature; 

			// Calculate the histogram
			calcHistInEllipseRGB( img, b_hist, g_hist, r_hist, 
					  			  feature.pt, feature.axes, feature.theta);

			EKF.init( a_feature->pt, a_feature->theta );
			EKF.update();
			last_update = nLastUpdate;
			match_id = nMatchID;
			predict_pt = feature.pt;
			predict_theta = feature.theta;
		}
		
		// update current tracker with matched feature
		// img - current frame image
		// feature - matched feature
		// nLastUpdate - current frame no.
		// nMatchID - 
		void update( const Mat img, OxfordFeature feature, int nLastUpdate, int nMatchID )
		{
			nSucces ++;
			nFail = 0;
			nTracked ++;

			Point pt = feature.pt;
			vTrj->push_back( pt - a_feature->pt );
			vAxes.push_back( feature.axes );	
			*a_feature = feature;
			// Calculate the histogram
			calcHistInEllipseRGB( img, b_hist, g_hist, r_hist, 
					  feature.pt, feature.axes, feature.theta);

			EKF.measure( feature.pt, feature.theta );
			EKF.update();
			last_update = nLastUpdate;
			match_id = nMatchID;
			EKF.output_pre( predict_pt, predict_theta );
		}
		
		void update_failure( long nFrame, Point Correct_pose, float fScale )
		{
			nSucces = 0;
			nFail ++;
			nTracked ++;
			last_update = nFrame;
			match_id = -1;

			Point pt;
			float theta;
			EKF.measure( estimate_pt, predict_theta );
			EKF.output_correct( pt, theta );
			vTrj->push_back( Correct_pose - a_feature->pt );
			a_feature->axes.width  = round( float( estimate_axes.width ) * float( fScale ) );
			//a_feature->axes.width  = a_feature->axes.width;
			a_feature->axes.height  = round( float( estimate_axes.height ) * float( fScale ) );
			//a_feature->axes.height = a_feature->axes.height;
			vAxes.push_back( a_feature->axes );	
			//a_feature->theta = theta;
			a_feature->pt = Correct_pose;

			EKF.update();
			EKF.output_pre( predict_pt, predict_theta );
		}

		float disimalarity( const OxfordFeature feature )
		{
			return abs( feature.cornerness - a_feature->cornerness );	
		}
		
		bool verify( const Mat img, const vector<TRACKER>* pTrackerList,
					 const OxfordFeature feature, long lFrame, float & fCurrentScale )
		{
			// color verification
			MatND dstb_hist, dstg_hist, dstr_hist;
			float fColorDist = colorVerify( img, feature, dstb_hist, dstg_hist, dstr_hist );
			
			// trajectory verification
			 
			float fTrajDist = 0.0;
			estimate_pt = predict_pt;
			//trajVerify( pTrackerList, lFrame );
			Point ptDist = estimate_pt - a_feature->pt;
			fTrajDist = sqrt( pow( ptDist.x, 2 ) + pow( ptDist.y, 2 ) );

			if( ( fColorDist < 0.8 )  /*  && ( fTrajDist < 10.0)*/ ) 
			{
				// copy the histogram info to current tracker
				dstb_hist.copyTo( this->b_hist );
				dstg_hist.copyTo( this->g_hist );
				dstr_hist.copyTo( this->r_hist );
				return true;
			}
			else
			{
				match_id = -1;
				return false;
			}

		}

		float trajVerify( const vector<TRACKER> * pTrackerList, int nFrameNum )		
		{
			vector<int> vAuxPoint;	// estimated trajectories
			vector<Point> vDist;
			vector<Point> vEstimate_pt;
			Point2f fEstimateSum = Point2f(0, 0);
			int nAuxTotal = 0;
			int MinDist = 500;
			Point MinVol = Point(0, 0);
			// Select auxiliary point
			for( int i = 0; i < pTrackerList->size(); i ++ )
			{
				// Judge is an auxiliary point.
				// obtain the latest trajectory
				vector<Point> vTraj1, vTraj2;
				bool bIsAux = false;	// auxiliary point flag

				// MinDist
				if( ( pTrackerList->at(i).nSucces > TRACKER_TRAJ_FRAME) && ( pTrackerList->at(i).id != this->id) && ( pTrackerList->at(i).last_update == nFrameNum  ) ) 
				{
					// Get the distance of the
					Point ptMinDist = pTrackerList->at(i).a_feature->pt - a_feature->pt;
					int nDist = sqrt( pow( ptMinDist.x, 2 ) + pow( ptMinDist.y, 2) );
					if( nDist < MinDist )
					{
						MinDist = nDist;
						int idx = pTrackerList->at(i).vTrj->size() - 1; 
						Point ptEstimate = pTrackerList->at(i).vTrj->at(idx);
						MinVol = ptEstimate;
					}

					// Get the current tracker latest trajectory
					int minFrame = vTrj->size() - TRACKER_TRAJ_FRAME;
					if( minFrame < 0 )
					{
						minFrame = 0;
					}
					for( int j = vTrj->size() - 1; j >= minFrame ; j -- )
					{
						Point pt;
						pt = vTrj->at(j);
						vTraj1.push_back(pt);
					}
					
					// Get the potential auxiliary tracker latest trajectory
					minFrame = pTrackerList->at(i).vTrj->size() - TRACKER_TRAJ_FRAME - 1;
					if( minFrame < 0 )
					{
						minFrame = 0;
					}
					for( int j = pTrackerList->at(i).vTrj->size() - 2; j >= minFrame; j -- )
					{
						Point pt;
						pt = pTrackerList->at(i).vTrj->at(j);
						vTraj2.push_back(pt);
					}

					// Compare two trajectories is relative or not.
					bIsAux = isAuxiliaryPoint( vTraj1, vTraj2 );
					// Generate the global scale factor and calculate the estimate position
					if( bIsAux )
					{
						vAuxPoint.push_back( i );
						Point ptDist1 = vTraj1.at(0) - vTraj2.at(0);
						//Point ptEstimate = pTrackerList->at(i).a_feature->pt + ptDist1;
						int idx = pTrackerList->at(i).vTrj->size() - 1; 
						Point ptEstimate = pTrackerList->at(i).vTrj->at(idx);
						vDist.push_back( ptDist1 );
						vEstimate_pt.push_back( ptEstimate );
					}
				}
			}
			
			
			Point2f ptSumEst = Point2f(0, 0);
			if( vEstimate_pt.size() > 0 )
			{
				for( int i = 0; i < vEstimate_pt.size(); i ++ )
				{
					ptSumEst.x += vEstimate_pt.at(i).x;
					ptSumEst.y += vEstimate_pt.at(i).y;
				}
				ptSumEst.x /= float(vEstimate_pt.size());
				ptSumEst.y /= float( vEstimate_pt.size() ); 
				ptSumEst.x = round(ptSumEst.x);
				ptSumEst.y = round(ptSumEst.y);
			}

			// Estimate global scale factor
			Point ptAxesChange = Point(0, 0);
			int	  nScaleTotal = 0;
			for( int i = 0; i < vAuxPoint.size(); i ++ )
			{
				int idx = vAuxPoint.at(i);
				int nSize = pTrackerList->at(idx).vAxes.size() - 1;
				int r1 = pTrackerList->at(idx).vAxes.at(nSize).width - pTrackerList->at(idx).vAxes.at(nSize - 1).width; 	
				int r2 = pTrackerList->at(idx).vAxes.at(nSize).height - pTrackerList->at(idx).vAxes.at(nSize - 1).height;
				ptAxesChange.x += r1;
				ptAxesChange.y += r2;
				nScaleTotal ++;
			}


			if( nScaleTotal == 0 )
			{
				ptAxesChange.x = 0;
				ptAxesChange.y = 0;

				estimate_scale = 1.0;
			}
			else
			{
				ptAxesChange.x /= nScaleTotal;
				ptAxesChange.y /= nScaleTotal;
				estimate_scale = 1.0;
			}
			
			Point2f corrPt;
			if( vDist.size() > 0 )
			{
				Point2f mean_est = Point2f( 0, 0 ); 
				for( int i = 0; i < vDist.size(); i ++ )
				{
					mean_est.x += vDist.at(i).x;
					mean_est.y += vDist.at(i).y;
				}

				mean_est.x /= float( vDist.size() );
				mean_est.y /= float( vDist.size() );

				//estimate_pt = ptSumEst;
				estimate_pt.x = int( a_feature->pt.x + ptSumEst.x );
				estimate_pt.y = int( a_feature->pt.y + ptSumEst.y );
				estimate_axes.width = a_feature->axes.width + ptAxesChange.x;
				estimate_axes.height = a_feature->axes.height + ptAxesChange.y;
			}
			else
			{
				estimate_axes = a_feature->axes;
				estimate_pt = predict_pt;
			}
			
			return 0.0;
			// Generate intesections of different trajectories
			/* 
			Point2f corrPt;
			if( vEstPoint.size() > 0 )
			{
				Point2f mean_est;
				mean_est.x = 0;
				mean_est.y = 0;
				for( int i = 0; i < vEstPoint.size(); i ++ )
				{
					mean_est.x += vEstPoint.at(i).x;	
					mean_est.y += vEstPoint.at(i).y;	
				}
				mean_est.x /= vEstPoint.size();	
				mean_est.y /= vEstPoint.size();	
				
				estimate_pt.x = int(mean_est.x);
				estimate_pt.y = int(mean_est.y) ;
			
				float dist = sqrt( pow( ( corrPt.x - estimate_pt.x ), 2)
					 		+ pow( ( corrPt.y - estimate_pt.y ), 2 ) ) ;
				return dist;
			}
			else
			{
				estimate_pt = predict_pt;
				return 0.0;
			}*/

		}

		bool isAuxiliaryPoint( const vector<Point> traj1, const vector<Point> traj2 )
		{
			// Normalize the trajectories
			Point mean_pt1, mean_pt2;
			mean_pt1.x = 0; mean_pt1.y = 0;
			mean_pt2.x = 0; mean_pt2.y = 0;
			for( int i = 0; i < traj1.size(); i ++ )
			{
				mean_pt1 += traj1.at(i);	
				mean_pt2 += traj2.at(i);	
			}
			mean_pt1.x = mean_pt1.x / traj1.size();
			mean_pt1.y = mean_pt1.y / traj1.size();
			mean_pt2.x = mean_pt2.x / traj1.size();
			mean_pt2.y = mean_pt2.y / traj1.size();
			
			vector<Point> vNormPt, vNorm2;	
			for( int i = 0; i < traj1.size(); i ++ )
			{
				Point pt1, pt2;
				pt1 = traj1.at(i);
				pt2 = traj2.at(i);
				vNormPt.push_back(pt1);
				vNormPt.push_back(pt2);
				vNorm2.push_back( pt2 );
			}

			// Convert Point sets to Mat
			Mat matTraj, matTraj2;
			convPointToMat( vNormPt, matTraj );
			convPointToMat( vNorm2, matTraj2 );
			// Calculate the covariance matrix & eigenvalues
			Mat matCov, matCov2;
			mulTransposed( matTraj, matCov, true, Mat(), 1, CV_32F );
			mulTransposed( matTraj2, matCov2, true, Mat(), 1, CV_32F );

			// Calculate eigenvalue matrix
			Mat matEigen, matEigenVector;
			Mat matEigen2, matEigenVector2;
			eigen( matCov, matEigen, matEigenVector );	
			eigen( matCov2, matEigen2, matEigenVector2 );	
			matEigen = cv::abs( matEigen );
			matEigen2 = cv::abs( matEigen2 );
			
			// Sort eigenvalues
			Mat matSorted, matSorted2;
			cv::sort( matEigen, matSorted, CV_SORT_EVERY_COLUMN + CV_SORT_DESCENDING );
			cv::sort( matEigen2, matSorted2, CV_SORT_EVERY_COLUMN + CV_SORT_DESCENDING );

			//cout<<matSorted<<endl;
			float pEig2[4];
			pEig2[0] = matSorted2.ptr<float>(0)[0];
			pEig2[1] = matSorted2.ptr<float>(1)[0];
			pEig2[2] = matSorted2.ptr<float>(2)[0];
			pEig2[3] = matSorted2.ptr<float>(3)[0];
			
			float minSigma = 10.00;
			for(int i = 0; i < 4 ; i ++ )
			{
				if( pEig2[i] < minSigma )
				{
					minSigma = pEig2[i];
				}
			}
			minSigma = pow( minSigma, 2 );
			float pEig[4];
			//cout<<"min"<<minSigma<<endl;
			pEig[0] = matSorted.ptr<float>(0)[0];
			pEig[1] = matSorted.ptr<float>(1)[0];
			pEig[2] = matSorted.ptr<float>(2)[0];
			pEig[3] = matSorted.ptr<float>(3)[0];
			//cout<<pEig[0]<<endl;
			//cout<<pEig[1]<<endl;
			//cout<<pEig[2]<<endl;
			//cout<<pEig[3]<<endl;
			// Compare to thresholds & return the judge results
			int nEigThresholdNum = 0;
			for(int i = 0; i < 4 ; i ++ )
			{
				if( pEig[i] > 1.0e-03 )
				{
					nEigThresholdNum ++;
				}
			}

			if( nEigThresholdNum > 2 )
			{
				return false;
			}
			else
			{
				return true;
			}
		}
		
		void convPointToMat( const vector<Point> pts, Mat & mat )
		{	
			mat = Mat::zeros( 2, pts.size(), CV_32F );
			for( int i = 0; i < pts.size(); i ++ )
			{
				mat.at<float>( 0, i ) = pts.at(i).x;
				mat.at<float>( 1, i ) = pts.at(i).y;
			}
		}
		float colorVerify( const Mat img, const OxfordFeature feature,
						   MatND & fb_hist, MatND & fg_hist, MatND & fr_hist )
		{
			calcHistInEllipseRGB( img, fb_hist, fg_hist, fr_hist, 
					 		 	  feature.pt, feature.axes, feature.theta);
			float fDistB = compareHist( b_hist, fb_hist, CV_COMP_BHATTACHARYYA );
			float fDistG = compareHist( g_hist, fg_hist, CV_COMP_BHATTACHARYYA );
			float fDistR = compareHist( r_hist, fr_hist, CV_COMP_BHATTACHARYYA );
			float fDist = ( fDistB + fDistG + fDistR ) / 3;

			return fDist;
		}

		void write_file( FileStorage &fs )
		{
			fs << "id" << int(id) ;
			fs << "position" << "{";
			fs << "x" << int( a_feature->pt.x );
			fs << "y" << int( a_feature->pt.y );
			fs << "}"; 
			fs << "axes" << "{";
			fs << "axis1" << int( a_feature->axes.width );
			fs << "axis2" << int( a_feature->axes.height );
			fs << "}";
			fs << "theta" << float( a_feature->theta );
			fs << "cornerness" << float( a_feature->cornerness );
			fs << "TrajNr" << int( vTrj->size() );
			
			Mat matTraj_x, matTraj_y;
			matTraj_x = Mat::zeros( 1, vTrj->size(), CV_32F );
			matTraj_y = Mat::zeros( 1, vTrj->size(), CV_32F );
			for( int i = 0; i < vTrj->size(); i ++ )
			{
				matTraj_x.at<float>( 0, i ) = vTrj->at(i).x;
				matTraj_y.at<float>( 0, i ) = vTrj->at(i).y;
			}
			fs << "Traj_x" << matTraj_x;
			fs << "Traj_y" << matTraj_y;
		}

		void draw( Mat & img, const Scalar color1 = Scalar( 0, 0, 255), const Scalar color2 = Scalar( 255, 0, 0 ), const Scalar color3 = Scalar( 0, 255, 0 ) )
		{
			if( this->match_id != -1 )
			{
				ellipse( img, a_feature->pt, a_feature->axes, a_feature-> theta, 0, 360, color1, 2);
			}
			else
			{
				ellipse( img, a_feature->pt, a_feature->axes, a_feature-> theta, 0, 360, color2, 2);
			}
			string strId = Int2String( id );	
			putText( img, strId, a_feature->pt,
					 FONT_HERSHEY_PLAIN, 0.8, color3 );
		}
}_TRACKER;

#endif
