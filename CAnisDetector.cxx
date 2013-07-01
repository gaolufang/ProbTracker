#include "CAnisDetector.h"

CAnisDetector::CAnisDetector()
{

}

CAnisDetector::~CAnisDetector()
{

}

void CAnisDetector::update( const Mat & img )
{
	switch( m_nMode )
	{
	case OXFORD_FILE_MODE:
		FeatureReadFromFile();
		break;
	default:
		break;
	}
}
