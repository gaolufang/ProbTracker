#ifndef _CSOURCE_H
#define _CSOURCE_H

#include <iostream>
#include <stdio.h>
#include <string>

#include "LGlbLib.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv/cv.h"

#define SOURCE_VIDEO 0
#define SOURCE_IMAGE 1

using namespace std;
using namespace cv;
using namespace GLBLIB;

class CSource
{
public:
	CSource();
	~CSource();

	void SetVideoFileName( string strFileName );
	void SetImageFilePrefix( string strFileName, string strFileType, int nStart, int nEnd, int nBit );
	bool isOpened();
	void init();
	Mat	 update();
	long FrameNum();

public:
	Mat 	m_mCurrentImg;
	string 	m_strVideoFileName;
	string 	m_strImgPrefix;
	string 	m_strImgType;
	bool	m_bIsOpened;
	long 	m_lFrameNum;
	long	m_nStart;
	long	m_nEnd;
	int		m_nMode;
	int		m_nBit;

private:
	VideoCapture m_cvCapture;
};

#endif
