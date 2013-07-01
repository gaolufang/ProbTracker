#include "CSource.h"

CSource::CSource()
{
	m_bIsOpened = false;
	m_nMode = -1;
}

CSource::~CSource()
{
	m_cvCapture.release();
}

void CSource::SetVideoFileName( string strFileName )
{
	m_nMode = SOURCE_VIDEO;
	m_strVideoFileName = strFileName;
}

void CSource::SetImageFilePrefix( string strFileName, string strFileType, int nStart, int nEnd, int nBit )
{
	m_nMode = SOURCE_IMAGE;
	m_strImgPrefix = strFileName;
	m_strImgType = strFileType;
	m_nStart = nStart;
	m_nEnd = nEnd;
	m_nBit = nBit;
}

void CSource::init()
{
	switch( m_nMode ) 
	{
		case SOURCE_VIDEO:
			{
				m_cvCapture.open( m_strVideoFileName );
				if( m_cvCapture.isOpened() )
				{
					m_lFrameNum = 0;
					m_bIsOpened = true;
					m_cvCapture.read(m_mCurrentImg) ;
					break;
				}
				else
				{
					m_bIsOpened = false;
					break;
				}
			}	
		case SOURCE_IMAGE:
			{
				m_lFrameNum = m_nStart;
				string strImgNum = Int2BitString( m_nStart, m_nBit);
				string strFullFileName  = m_strImgPrefix + strImgNum + m_strImgType; 
				cout<<strFullFileName<<endl;
				m_mCurrentImg = imread( strFullFileName );
				if ( ( m_mCurrentImg.rows > 0 ) && ( m_mCurrentImg.cols > 0) )
				{
					m_bIsOpened = true;
				}
				else
				{
					m_bIsOpened = false;
				}
				break;
			}		
		default:
			{
				break;
			}
	}
}

Mat CSource::update()
{
	switch( m_nMode ) 
	{
		case SOURCE_VIDEO:
			{
				if( m_cvCapture.read(m_mCurrentImg) )
				{
					m_lFrameNum ++;
					m_bIsOpened = true;
					return m_mCurrentImg;
				}
				else
				{
					m_bIsOpened = false;
					return m_mCurrentImg;
					break;
				}
			}	
		case SOURCE_IMAGE:
			{
				string strImgNum = Int2BitString( m_lFrameNum, m_nBit );
				string strFullFileName  = m_strImgPrefix + strImgNum + m_strImgType; 
				m_mCurrentImg = imread( strFullFileName );
				m_lFrameNum ++;
				if ( ( m_mCurrentImg.rows > 0 ) && ( m_mCurrentImg.cols > 0) 
						&& ( m_lFrameNum <= m_nEnd ) )
				{
					m_bIsOpened = true;
					return m_mCurrentImg;
				}
				else
				{
					m_bIsOpened = false;
					return m_mCurrentImg;
				}
				break;
			}		
		default:
			{
				break;
			}
	}

}

bool CSource::isOpened()
{
	return m_bIsOpened;
}

long CSource::FrameNum()
{
	return m_lFrameNum;
}
