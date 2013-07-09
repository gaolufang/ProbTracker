#include "CTracker.h"
/** 
 * Initialize the parameters of Trackerlist 
 */
CTracker::CTracker()
{
	m_lIdCnt 	= 0;
	m_lFrameCnt = 0;
	m_lInitTrackerNum = 0;
	m_lCurrTrackerNum = 0;
	m_nLostThreshold = 20;
	m_ptScaleFactor.x = 1.0f;
	m_ptScaleFactor.y = 1.0f;
	m_fGlobalScale = 1.0f;
	m_fOverlapThresh = 0.4f;
	m_fCornernessThresh = 0.4f;
	m_pTrackerList = new vector<TRACKER>;
}

/**
 * Release the memory and de-construct the class
 */
CTracker::~CTracker()
{
	m_pTrackerList->clear();
}

/** 
 * Import updated feature list
 * @param pFeatureList		the import feature list	
 */
void CTracker::AIAFeatureInput( vector<OxfordFeature>* pFeatureList )
{
	/* Setup the feature type */
	m_nFeatureType = FEATURE_AIA;
	m_pAIAFeatureList = pFeatureList;
}

/**
 * Extended Kalman filter based tracking process
 * @param img		the image of current frame
 */
void CTracker::EKF_update( const Mat & img )
{
	m_nTrackerType = TRACK_KF;
	m_lFrameCnt ++;
	
	m_cvSize.width = img.cols;
	m_cvSize.height = img.rows;

	if( m_pTrackerList->size() == 0 ) 
	{
		m_pTrackerList->clear();
		img.copyTo( m_matCurrentImg );
		this->init();
		m_lInitTrackerNum = m_pTrackerList->size();
	}
	else
	{
		m_matCurrentImg.copyTo( m_matPreviousImg );
		img.copyTo( m_matCurrentImg );
		float fScaleSum = 0.0;
		int	  nScaleNum = 0;
		m_ptScaleFactor.x = 1.0;
		m_ptScaleFactor.y = 1.0;
		m_fGlobalScale = 1.0f;
		for( int i = 0; i < m_pTrackerList->size(); i ++ )
		{
			TRACKER* pTracker = &m_pTrackerList->at(i);
			pTracker->match_id = -1;
			OxfordFeature* pTrackerFeature = pTracker->a_feature;
			pTracker->isSalient = false;
			
			vector<MATCHRESULT> vMatchList;
			float fMaxDisim = 0;
			for( int j = 0; j < m_pAIAFeatureList->size(); j ++ )
			{
				OxfordFeature * pDetectedFeature = &m_pAIAFeatureList->at(j);

				bool bMatch;
				MATCHRESULT matchResult;
				bMatch = compareFeatureToTracker( pTracker, pDetectedFeature, matchResult, i, j );
				if( bMatch )
				{
					vMatchList.push_back( matchResult );
					if( matchResult.cornerDiff > fMaxDisim )
					{
						fMaxDisim = matchResult.cornerDiff;
					}
				}
				else
				{
					continue;
				}
			}
		
			// Find the best match in the search region
			int nMatchID = updateBestMatch( pTracker, vMatchList, fMaxDisim );

			float fCurrentScale = 1.0;	
			if( ( nMatchID != -1 ) ) 
			{
				if( pTracker->verify( img, m_pTrackerList, m_pAIAFeatureList->at(nMatchID), m_lFrameCnt, fCurrentScale ) )	
				{
					pTracker->update( img, m_pAIAFeatureList->at(nMatchID), m_lFrameCnt, nMatchID );
					m_pAIAFeatureList->erase(m_pAIAFeatureList->begin() + nMatchID, 
									  		 m_pAIAFeatureList->begin() + nMatchID + 1);

				}
			}
		}
	}
	//this->TrajAnalysis();
	
	// Motion correction
	m_fGlobalScale = 1.0;
	for( int i = 0; i < m_pTrackerList->size(); i ++ )
	{
		if( m_pTrackerList->at(i).last_update != m_lFrameCnt )
		{
			m_pTrackerList->at(i).trajVerify( m_pTrackerList, m_lFrameCnt );
			m_pTrackerList->at(i).update_failure( m_lFrameCnt, m_pTrackerList->at(i).predict_pt, m_fGlobalScale );
		}
	}

	this->clean();
	m_lCurrTrackerNum = m_pTrackerList->size();
	float fRatio = float( m_lCurrTrackerNum ) / float( m_lInitTrackerNum );
	if( fRatio < 0.8 )
	{
		for( int i = 0; i < m_pAIAFeatureList->size(); i ++ )
		{
			bool bMathed = false;
			for( int j = 0; j < m_pTrackerList->size(); j ++ )
			{
				if( i == m_pTrackerList->at(j).match_id )
				{
					bMathed = true;
					break;
				}
			}

			if( bMathed == false )
			{
				m_lIdCnt ++;
		
				TRACKER tracker( m_matCurrentImg, m_lIdCnt, m_lFrameCnt, i, m_pAIAFeatureList->at(i) );
				m_pTrackerList->push_back(tracker);
			}
		}
		m_lInitTrackerNum = m_pTrackerList->size();			
	}
}

/**
 * When the number of the trackers meet initialize condition, will re-initialize 
 * the tracker.
 */
void CTracker::init()
{
	switch( m_nFeatureType )
	{
		case FEATURE_AIA:
			for( int i = 0; i < m_pAIAFeatureList->size(); i ++ )
			{
				m_lIdCnt ++;
		
				TRACKER tracker( m_matCurrentImg, m_lIdCnt, m_lFrameCnt, i, m_pAIAFeatureList->at(i) );
				m_pTrackerList->push_back(tracker);
			}
			break;
		
		defaut:
		;
	}
}

/** 
 * Clean the list of trackers, if tracker failed in the certain frame.
 */
void CTracker::clean()
{
	for( int i = m_pTrackerList->size() - 1; i >=0; i -- )
	{
		if ( m_pTrackerList->at(i).match_id == -1 )
		{
			if( m_pTrackerList->at(i).nFail > m_nLostThreshold )
			{ 
				m_pTrackerList->erase(m_pTrackerList->begin() + i, 
									  m_pTrackerList->begin() + i + 1);
			}
		}
	}
}
/** 
 * Draw the tracker's status on a image frame.
 * @img		targeted image frame.
 * @color2  color for successful tracker 
 * @color2  color for failed tracker 
 */
void CTracker::draw( Mat& img, const Scalar color1, const Scalar color2, const Scalar color3 )  
{
	for( int i = 0; i < m_pTrackerList->size(); i ++ )
	{
		m_pTrackerList->at(i).draw( img, color1, color2, color3 );
	}
}
/**
 * Output the result to the XML file.
 * @strFileName		filename of the output list
 */
void CTracker::write_file( const string strFileName )
{
	FileStorage fs( strFileName, FileStorage::WRITE );
	
	int nTrackerNum = m_pTrackerList->size();
	fs << "num_of_tracker" <<  nTrackerNum;

	for( int i = 0; i < m_pTrackerList->size(); i ++ )
	{
		TRACKER * pTracker = &m_pTrackerList->at(i);
		fs << "Tracker";
		fs << "{";
		pTracker->write_file( fs );	
		fs << "}";
	}
	
	fs.release();
}
/**
 * Output the result to the text file.
 * @strFileName		filename of the output list
 */
void CTracker::write_txt_file( const string strFilename )
{
	ofstream file;
	file.open( strFilename.c_str() );
	file<<"id"<<"\t"<<"x"<<"\t"<<"y"<<"\t"<<"r1"<<"\t"<<"r2"<<"\t"<<"theta"<<"\r";
	for( int i = 0; i < m_pTrackerList->size() ; i ++ )
	{
		OxfordFeature * feature = m_pTrackerList->at(i).a_feature;
		file<<m_pTrackerList->at(i).id<<"\t"
			<<feature->pt.x<<"\t"
			<<feature->pt.y<<"\t"
			<<feature->axes.width<<"\t"
			<<feature->axes.height<<"\t"
			<<feature->theta<<"\r";
	}
	file.close();
}
/**
 * Find a specific tracker by id, and draw on image.
 * @nId 	the id of the selected tracker.
 * @img	    the image want to be drew on.
 */
void CTracker::findById( int nId, Mat img )
{
	if( m_pTrackerList->size() > 0 )
	{
		bool bFound = false;
		for( int i = 0; i < m_pTrackerList->size(); i ++ )
		{
			if( m_pTrackerList->at(i).id == nId )
			{
				bFound = true;
				m_pTrackerList->at(i).draw( img );
				cout<<"Tracker Found."<<endl;
			}
		}

		if( !bFound )
		{
			cout<<"Tracker has not found."<<endl;
		}
	}
}

void CTracker::TrajAnalysis()
{
	int nScaleFactorNum = 0;
	m_ptScaleFactor.x = 0.0;
	m_ptScaleFactor.y = 0.0;
	float fScalFactorSum = 0;
	for( int i = 0; i < m_pTrackerList->size(); i ++ )
	{
		TRACKER * pTracker1 = & m_pTrackerList->at(i);
		if( pTracker1->isSalient == false && pTracker1->nSucces >= CTRACKER_TRAJ_FRAME )
		{
			vector<Point> vTraj1, vTraj2;
			for( int j = pTracker1->vTrj->size() - 1; j > pTracker1->vTrj->size() - CTRACKER_TRAJ_FRAME - 1; j -- )
			{
				Point pt = pTracker1->vTrj->at( j );
				vTraj1.push_back( pt );
			}

			for( int j = i + 1; j < m_pTrackerList->size(); j ++ )
			{
				TRACKER * pTracker2 = & m_pTrackerList->at(j);

				if( ( pTracker1->nTracked > CTRACKER_TRAJ_FRAME ) 
				 && ( pTracker2->nTracked > CTRACKER_TRAJ_FRAME ) 
				 && ( pTracker2->isSalient == false ) 
				 && ( pTracker2->nSucces >= CTRACKER_TRAJ_FRAME ) )
				{
					for( int k = pTracker2->vTrj->size() - 1; k > pTracker2->vTrj->size() - CTRACKER_TRAJ_FRAME - 1; k -- )
					{
						Point pt = pTracker2->vTrj->at( k );
						vTraj2.push_back( pt );
					}
					
					if( true )
					{
						pTracker1->isSalient = true;
						pTracker2->isSalient = true;

						// Get the distance of two trackers & calculate scale factor
						Point ptDist1 = pTracker1->a_feature->pt - pTracker2->a_feature->pt;
						Point ptDist2 = ( pTracker1->a_feature->pt + vTraj1.at(1) ) - ( pTracker2->a_feature->pt + vTraj2.at(1) );
						float fDist1 = sqrt( pow( ptDist1.x, 2 ) + pow( ptDist1.y, 2) );
						float fDist2 = sqrt( pow( ptDist2.x, 2 ) + pow( ptDist2.y, 2) );
						if( fDist2 != 0 && fDist1 != 0 )
						{
							fScalFactorSum += fDist1 / fDist2;
							nScaleFactorNum ++;
						}
					}
				}
			}
		}
	}
		
	// Generate global scale factor
	cout<<nScaleFactorNum<<endl;
	if( nScaleFactorNum == 0 )
	{
		m_ptScaleFactor.x = 1.0f;
		m_ptScaleFactor.y = 1.0f;
		fScalFactorSum = 1.0f;
	}
	else
	{
		fScalFactorSum /= float( nScaleFactorNum );
		fScalFactorSum =  1 + 4 * abs( 1 - fScalFactorSum ) ;
		if( fScalFactorSum > 3 || fScalFactorSum < 1.0 )
		{
			fScalFactorSum = 1.0;
		}
	}

	m_fGlobalScale = fScalFactorSum;
	cout<<m_fGlobalScale<<endl;
	// Generate hypothesis postion
	for( int i = 0; i < m_pTrackerList->size(); i ++ )
	{
		TRACKER * pTrackerObj = &m_pTrackerList->at(i);
		Point ptHypoSum = Point(0, 0);
		int nHypoNum = 0;

		if( pTrackerObj->last_update != m_lFrameCnt )
		{
			for( int j = 0; j < m_pTrackerList->size(); j ++ )
			{
				if( i != j )
				{
					TRACKER * pTrackerAux = &m_pTrackerList->at(j);
					if( pTrackerAux->isSalient == true )
					{
						Point2f ptLastAuxPt = pTrackerAux->vTrj->at( pTrackerAux->vTrj->size() - 2 );
						Point2f ptLastPt;
						if( pTrackerObj->last_update != m_lFrameCnt )
						{
							ptLastPt = pTrackerObj->a_feature->pt;	
						}
						else if( pTrackerObj->isSalient == false )
						{
							ptLastPt = pTrackerObj->vTrj->at( pTrackerObj->vTrj->size() - 2 );
						}
						Point2f ptDiff = ptLastPt - ptLastAuxPt;
						ptDiff.x *= m_fGlobalScale;
						ptDiff.y *= m_fGlobalScale;
					
						Point2f ptHypo;
						ptHypo.x = ptDiff.x + float( pTrackerAux->a_feature->pt.x );
						ptHypo.y = ptDiff.y + float( pTrackerAux->a_feature->pt.y );
					
						ptHypoSum.x += ptHypo.x;
						ptHypoSum.y += ptHypo.y;
						nHypoNum ++;
					}
				}

			}

			Point ptCorrect;
			if( nHypoNum != 0 )
			{
				ptHypoSum.x /= nHypoNum;
				ptHypoSum.y /= nHypoNum;
				ptCorrect.x = ptHypoSum.x;
				ptCorrect.y = ptHypoSum.y;
				//pTrackerObj->update_failure( m_lFrameCnt, ptCorrect, m_fGlobalScale );
			}
			else
			{
				ptHypoSum = pTrackerObj->predict_pt;
				ptCorrect.x = ptHypoSum.x;
				ptCorrect.y = ptHypoSum.y;
				//pTrackerObj->update_failure( m_lFrameCnt, ptCorrect, m_fGlobalScale );
			}
			//cout<<"Cor"<<ptCorrect<<endl;
		}	
	}
}

bool CTracker::isAuxiliaryPoint( const vector<Point> traj1, const vector<Point> traj2 )
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
	
	vector<Point> vNormPt;	
	for( int i = 0; i < traj1.size(); i ++ )
	{
		Point pt1, pt2;
		pt1 = traj1.at(i) - mean_pt1;
		pt2 = traj2.at(i) - mean_pt2;
		vNormPt.push_back(pt1);
		vNormPt.push_back(pt2);
	}

	// Convert Point sets to Mat
	Mat matTraj;
	convPointToMat( vNormPt, matTraj );
	
	// Calculate the covariance matrix & eigenvalues
	Mat matCov, matSort;
	//calcCovarMatrix( matTraj, matCov, matMean, CV_COVAR_NORMAL | CV_COVAR_COLS, CV_32F );
	mulTransposed( matTraj, matCov, true, Mat(), 1, CV_32F );
	//cout<<matCov<<endl;
	// Calculate eigenvalue matrix
	Mat matEigen, matEigenVector;
	eigen( matCov, matEigen, matEigenVector );
	matEigen = cv::abs( matEigen );
	cv::sort( matEigen, matSort, CV_SORT_EVERY_COLUMN + CV_SORT_DESCENDING );
	//cout<<matSort<<endl;
	float pEig[4];
	pEig[0] = matSort.ptr<float>(0)[0];
	pEig[1] = matSort.ptr<float>(1)[0];
	pEig[2] = matSort.ptr<float>(2)[0];
	pEig[3] = matSort.ptr<float>(3)[0];
	int nEigThresholdNum = 0;
	for(int i = 0; i < 4 ; i ++ )
	{
		if( pEig[i] > 1000.0 )
		{
			nEigThresholdNum ++;
		}
	}
	// Compare to thresholds & return the judge results
	if( nEigThresholdNum > 2 )
	{
		return false;
	}
	else
	{
		return true;
	}

}

void CTracker::convPointToMat( const vector<Point> pts, Mat & mat )
{	
	mat = Mat::zeros( 2, pts.size(), CV_32F );
	for( int i = 0; i < pts.size(); i ++ )
	{
		mat.at<float>( 0, i ) = pts.at(i).x;
		mat.at<float>( 1, i ) = pts.at(i).y;
	}
}

/**
 * Calculate the correspondeness of the tracker with feature, 
 * return a sum score of the overlap error and cornerness error.
 * @pTracker	The tracker pointer
 * @pFeature	The feature pointer
 * @result		Return the match result
 */
bool CTracker::compareFeatureToTracker( TRACKER *pTracker, OxfordFeature *pFeature, MATCHRESULT &result , int idx_tracker, int idx_feature )
{
	OxfordFeature *pTrackerFeature = pTracker->a_feature;
	int nLongAxis = ( pTracker->a_feature->axes.width > pTracker->a_feature->axes.height )
					? pTrackerFeature->axes.width : pTrackerFeature->axes.height;
	int nSearchRange = nLongAxis ^ 2;

	Point ptDist = pTracker->predict_pt - pFeature->pt;
	int nDist = ptDist.x^2 + ptDist.y^2;
	
	if(nDist > nSearchRange)
	{
		return false;
	}

	result.dist = nDist;
	result.feature_id = idx_feature;
	result.tracker_id = idx_tracker;
	result.cornerDiff = pTracker->disimalarity( *pFeature );
	float fScale_w = 1.0f, fScale_h = 1.0f;
	float fOverlap, fUnion;
	Size szTrackerSize;
	szTrackerSize.width = pTrackerFeature->axes.width * fScale_w; 
	szTrackerSize.height = pTrackerFeature->axes.height * fScale_h; 
	fUnion = TwoEllipsesUnionArea( pTracker->predict_pt, szTrackerSize, pTrackerFeature->theta, pFeature->pt, pFeature->axes, pFeature->theta, fOverlap );
	result.overlap = 1.0 - ( fOverlap / fUnion );
	
	return true;
}

/**
 * Update the tracker match info with best match feature, return the index of feature list
 * @pTracker	the selected Trackerlist
 * @vMatchList  the match infomation
 * @fMaxDisim	the maximum error from cornerness
 */
int CTracker::updateBestMatch( TRACKER *pTracker, vector<MATCHRESULT> vMatchList, float fMaxDisim )
{
	int nMatchID = -1;
	float fMatchScore = 3.0f;
	
	for( int i = 0; i < vMatchList.size(); i ++ )	
	{
		vMatchList.at(i).cornerDiff /= fMaxDisim;
		if( ( vMatchList.at(i).cornerDiff < m_fCornernessThresh ) && ( vMatchList.at(i).cornerDiff < m_fOverlapThresh ) )
		{
			float score = vMatchList.at(i).cornerDiff + vMatchList.at(i).overlap;
			if( score < fMatchScore )
			{
				nMatchID = vMatchList.at(i).feature_id;
				fMatchScore = score;
			}
		}
	}

	return nMatchID;
}
