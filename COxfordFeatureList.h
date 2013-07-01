#ifndef _CLASS_FEATURE_H
#define _CLASS_FEATURE_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <stdio.h>

#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv/cxcore.h"

#include "LGlbLib.h"

/*  Modes List */
#define OXFORD_FILE_MODE 		0
#define OXFORD_DETETOR_MODE	 	1

using namespace std;
using namespace cv;
using namespace GLBLIB;

class COxfordFeatureList
{
	/* Functions List */
public:
	void SetMode( int mode );
	void SetFileType( string filetype );
	void SetFilePrefix( string prefix, string filetype, int start, int end );
	void filter( const Mat & img, int size, float threshold, int nNumFeature );
	bool ReadFileNames( string strFileName );
	void update( const Mat & img );
	void draw( Mat & img, const Scalar color );
	vector<OxfordFeature> * OutputList();
	void FeatureReadFromFile();

	COxfordFeatureList();
	~COxfordFeatureList();

	/* Variables List */ 
public:
	vector<OxfordFeature> * m_vFeatureList;
	vector<OxfordFeature> * m_vBackupFeatureList;
	int						m_nListLength;
	int						m_nCurrentFileNum;
	int						m_nMode;
	string					m_strCurrentFileName;
	string 					m_strFilePrefix;
	string					m_strFileType;
	int    					m_nStart, m_nEnd;
};

#endif
