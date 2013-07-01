#ifndef _CLASS_OPENPRJ_H
#define _CLASS_OPENPRJ_H

#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

class COpenPrj
{
	/* Member Function List */
public:
	void SetProjectFile( string strProjectFilename );
	void ReadDatasetConfig( string strDatasetCfgFilename );
	void ReadProjectConfig( string strPrjCfgFilename );
	string ImgPrefix()
	{
		return m_strImagePrefix;
	}
	int ImgBegin()
	{
		return m_nImageStart;
	}
	int ImgEnd()
	{
		return m_nImageEnd;
	}
	int ImgBit()
	{
		return m_nImageBit;
	}
	string ImgType()
	{
		return m_strImageType;
	}
	string FeaturePrefix()
	{
		return m_strFeaturePrefix;
	}
	string FeatureType()
	{
		return m_strFeatureType;
	}
	int FeatureBegin()
	{
		return m_nFeatureStart;
	}
	int FeatureEnd()
	{
		return m_nFeatureEnd;
	}
	string OutputFilePrefix()
	{
		return m_strOutputFilePrefix;
	}
	string OutputImgPrefix()
	{
		return m_strOutputImagePrefix;
	}

	COpenPrj();
	COpenPrj( string strProjectFilename );
	~COpenPrj();

private:
	int ProjectKeyName( string paraName );
	int DatasetKeyName( string paraName );

	/* Member Variable List  */
public:
	string	m_strProjectFilename;
	string 	m_strImagePrefix;
	string  m_strImageType;
	int		m_nImageStart;
	int		m_nImageEnd;
	int		m_nImageBit;
	string 	m_strFeaturePrefix;
	string  m_strFeatureType;
	int		m_nFeatureStart;
	int		m_nFeatureEnd;
	string  m_strOutputFilePrefix;
	string  m_strOutputImagePrefix;
};

#endif
