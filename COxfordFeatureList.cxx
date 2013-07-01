#include "COxfordFeatureList.h"

COxfordFeatureList::COxfordFeatureList()
{
	
}

COxfordFeatureList::~COxfordFeatureList()
{
	/* clear the feature list, release the memory */
	m_vFeatureList->clear();
	m_vBackupFeatureList->clear();
}

void COxfordFeatureList::SetFilePrefix(string prefix, string filetype, int start, int end)
{
	m_nMode 				= OXFORD_FILE_MODE;
	m_vFeatureList 			= new vector<OxfordFeature>;
	m_vBackupFeatureList 	= new vector<OxfordFeature>;
	m_strFilePrefix 		= prefix;
	m_strFileType			= filetype;
	m_nStart 				= start;
	m_nEnd 	 				= end;
	m_nCurrentFileNum 		= start - 1;
}

void COxfordFeatureList::SetFileType( string filetype )
{
	m_strFileType = filetype;
}

void COxfordFeatureList::SetMode( int mode )
{
	m_nMode = mode;
}

void COxfordFeatureList::draw( Mat & img, const Scalar color )
{
	for( int i = 0; i < m_vBackupFeatureList->size(); i ++ )
	{
		ellipse(  img, m_vBackupFeatureList->at(i).pt, m_vBackupFeatureList->at(i).axes, 
				  m_vBackupFeatureList->at(i).theta,	0, 360, color );
	}
}

void COxfordFeatureList::filter( const Mat& img, int size = 4, float thresh = 0.0, int nNumFeature = 100 )
{
	// Sort the list
	vector<OxfordFeature> * vSortedList = new vector<OxfordFeature>;
	while( m_vFeatureList->size() > 0 && vSortedList->size() < nNumFeature )
	{
		float maxCornerness = 0;
		int idxMax = -1;
		for( int i = 0; i < m_vFeatureList->size(); i ++ )
		{
			if( maxCornerness < m_vFeatureList->at(i).cornerness )
			{
				idxMax = i;
				maxCornerness = m_vFeatureList->at(i).cornerness;
			}
		}
		OxfordFeature feature = m_vFeatureList->at(idxMax);
		vSortedList->push_back(feature);
		m_vFeatureList->erase( m_vFeatureList->begin() + idxMax, 
							   m_vFeatureList->begin() + idxMax + 1 );		
	}	
	m_vFeatureList->clear();
	m_vBackupFeatureList->clear();
	m_vFeatureList = vSortedList;

	Mat mFeatureImg = Mat::zeros( img.rows, img.cols, CV_32F );
    OxfordFeature featureMatrix[img.rows][img.cols];

	// Initiaize the feature strength image and feature parameter matrix -- OpenCV edition
	for( int i = 0; i < m_vFeatureList->size(); i ++ )
	{
		mFeatureImg.at<float>( m_vFeatureList->at(i).pt.y, m_vFeatureList->at(i).pt.x )
			= m_vFeatureList->at(i).cornerness;
		featureMatrix[m_vFeatureList->at(i).pt.y][m_vFeatureList->at(i).pt.x]
			= m_vFeatureList->at(i);
	}
	
	// nonmaxsupres_opencv
	Mat mFilterImg;
	vector<Point> vPts;
	nonmaxsupres_opencv( mFeatureImg, mFilterImg, vPts, thresh, size );

	//Regenerate the feature list
	m_vFeatureList->clear();
	for( int i = 0; i < vPts.size(); i ++ )
	{
		OxfordFeature aFeature = featureMatrix[vPts.at(i).y][vPts.at(i).x];
		m_vFeatureList->push_back(aFeature);
		m_vBackupFeatureList->push_back(aFeature);
	}
		
	mFeatureImg.release();
}

vector<OxfordFeature> * COxfordFeatureList::OutputList()
{
	return m_vFeatureList;
}

void COxfordFeatureList::FeatureReadFromFile()
{
	m_nCurrentFileNum ++;
	m_vFeatureList->clear();
	string strNum = Int2String( m_nCurrentFileNum );
	string strFullFileName = 	m_strFilePrefix
							+	strNum
							+	m_strFileType;
	
	ifstream file;
	file.open( strFullFileName.c_str() );
	m_strCurrentFileName = strFullFileName;
	
	if( !file )
	{
		cout<<strFullFileName<<endl;
		cout<<"File open failed."<<endl;
	}

	string strLine;
	long m_nLineNum = 0;
	while( getline(file, strLine, '\r') )
	{
		int nLength = strLine.length();
		int nNumCnt = 0;
		string strNum = "";
		float readout[9];
		for (int i = 0; i < nLength; i ++)
		{
			strNum += strLine.at(i);

			if( (strLine.at(i) == '\t') || (i == nLength-1) )  
			{
				readout[nNumCnt] = atof( strNum.c_str() );
				nNumCnt ++;
				strNum = "";
			}
		}
		if ( (m_nLineNum != 0) && (nNumCnt == 9) )
		{
			OxfordFeature feature;
			feature.pt.x = int( readout[0] );
			feature.pt.y = int( readout[1] );
			feature.A = readout[2];
			feature.B = readout[3];
			feature.C = readout[4];

			feature.cornerness 	= readout[5];
			feature.axes.width	= readout[6];
			feature.axes.height	= readout[7];
			feature.theta		= ( readout[8] * 180 ) / 3.1415 ;
			m_vFeatureList->push_back( feature );
		}
		m_nLineNum ++;
	}

	file.close();
}
