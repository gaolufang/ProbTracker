#include "COpenPrj.h"

COpenPrj::COpenPrj()
{

}

COpenPrj::COpenPrj( string strProjectFilename )
{
	m_strProjectFilename = strProjectFilename;
	ReadProjectConfig( m_strProjectFilename );
}

COpenPrj::~COpenPrj()
{

}

void COpenPrj::SetProjectFile( string strProjectFilename )
{
	m_strProjectFilename = strProjectFilename;
	ReadProjectConfig( m_strProjectFilename );
}

void COpenPrj::ReadProjectConfig( string strPrjCfgFilename )
{
	ifstream file;
	file.open( strPrjCfgFilename.c_str() );

	string strLine;
	long nLineNum = 0;
	while( getline( file, strLine, '\n' ) )
	{
		if( nLineNum == 0 )
		{
			if( strLine.compare( "[project configure]" ) != 0  )
			{
				cout<<"Invalid project file."<<endl;
				break;
			}	
		}
		else
		{
			int loc = 0; string paraName = ""; char chLoc = strLine.at(0);
			while( chLoc != '='  )
			{
				paraName += strLine.at(loc);
				loc ++;
				chLoc = strLine.at(loc);
			}
			
			switch( ProjectKeyName( paraName ) )
			{
			case 0:
			{
				loc += 2; 
				chLoc = strLine.at(loc); 
				string prjPath = "";
				while( loc != strLine.length() - 1 )
				{
					prjPath += strLine.at(loc);
					loc ++;
					chLoc = strLine.at(loc); 
				}
				
				m_strOutputFilePrefix = prjPath + "/file/frame";
				m_strOutputImagePrefix = prjPath + "/img/frame";
				break;
			}

			case 1:
			{
				loc += 2; 
				chLoc = strLine.at(loc); 
				string prjPath = "";
				while( loc != strLine.length() - 1 )
				{
					prjPath += strLine.at(loc);
					loc ++;
					chLoc = strLine.at(loc); 
				}

				ReadDatasetConfig( prjPath );
				break;
			}

			}
		}

		nLineNum ++;
	}
}

int COpenPrj::ProjectKeyName( string paraName )
{
	if( paraName.compare( "project_path" ) == 0 )
	{
		return 0;
	}

	if( paraName.compare( "dataset_config" ) == 0 )
	{
		return 1;
	}
}

void COpenPrj::ReadDatasetConfig( string strDatasetCfgFilename )
{
	ifstream file;
	file.open( strDatasetCfgFilename.c_str() );

	string strLine;
	long nLineNum = 0;
	string imgPath, imgPrefix, featurePath, featurePrefix;
	while( getline( file, strLine, '\n' ) )
	{
		if( nLineNum == 0 )
		{
			if( strLine.compare( "[configure file]" ) != 0  )
			{
				cout<<"Invalid dataset configure file."<<endl;
				break;
			}	
		}
		else
		{
			int loc = 0; string paraName = ""; char chLoc = strLine.at(0);
			while( chLoc != '='  )
			{
				paraName += strLine.at(loc);
				loc ++;
				chLoc = strLine.at(loc);
			}
			
			cout<<paraName<<endl;
			loc ++; 
			chLoc = strLine.at(loc); 
			string strPath = "";
			while( loc < strLine.length() )
			{
				strPath += strLine.at(loc);
				loc ++;
				if ( loc < strLine.length() )
				{
					chLoc = strLine.at(loc); 
				}
			}
			
			switch( DatasetKeyName( paraName ) )
			{
			case 0:
			{
				imgPath = strPath;
				break;
			}

			case 1:
			{
				imgPrefix = strPath;
				break;
			}

			case 2:
			{
				m_nImageStart = atoi( strPath.c_str() );
				break;
			}

			case 3:
			{
				m_nImageEnd = atoi( strPath.c_str() );
				break;
			}

			case 4:
			{
				m_nImageBit = atoi( strPath.c_str() );
				break;
			}

			case 5:
			{
				m_strImageType = strPath;
				break;
			}

			case 6:
			{
				featurePath = strPath;
				break;
			}

			case 7:
			{
				featurePrefix = strPath;
				break;
			}

			case 8:
			{
				m_nFeatureStart = atoi( strPath.c_str() );
				break;
			}
			case 9:
			{
				m_nFeatureEnd = atoi( strPath.c_str() );
				break;
			}

			case 10:
			{
				m_strFeatureType = strPath;
				break;
			}

			}
		}

		nLineNum ++;
	}

	m_strImagePrefix = imgPath + "/" + imgPrefix;
	m_strFeaturePrefix = featurePath + "/" + featurePrefix;
	cout<<m_strImagePrefix<<endl;
	cout<<m_strFeaturePrefix<<endl;
}

int COpenPrj::DatasetKeyName( string paraName )
{
	if( paraName.compare( "image_path" ) == 0 )
	{
		return 0;
	} 
	
	if( paraName.compare( "image_prefix" ) == 0 )
	{
		return 1;
	}
	
	if( paraName.compare( "image_start" ) == 0 )
	{
		return 2;
	}
	
	if( paraName.compare( "image_end" ) == 0 )
	{
		return 3;
	}
	
	if( paraName.compare( "image_bit" ) == 0 )
	{
		return 4;
	}
	
	if( paraName.compare( "image_type" ) == 0 )
	{
		return 5;
	}
	
	if( paraName.compare( "feature_path" ) == 0 )
	{
		return 6;
	} 
	
	if( paraName.compare( "feature_prefix" ) == 0 )
	{
		return 7;
	}
	
	if( paraName.compare( "feature_start" ) == 0 )
	{
		return 8;
	}
	
	if( paraName.compare( "feature_end" ) == 0 )
	{
		return 9;
	}
	
	if( paraName.compare( "feature_type" ) == 0 )
	{
		return 10;
	}
	
}
