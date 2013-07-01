#include "C2DOFKF.h"

C2DOFKF::C2DOFKF()
{
	const float pT[] = {   1, 0, 0, 1, 0, 0,
				     	   0, 1, 0, 0, 1, 0,
					 	   0, 0, 1, 0, 0, 1,
						   0, 0, 0, 1, 0, 0, 
						   0, 0, 0, 0, 1, 0,
			 		 	   0, 0, 0, 0, 0, 1 };	// Transformation matrix
	
	const float pM[] = {   1, 0, 0, 0, 0, 0,
				     	   0, 1, 0, 0, 0, 0,
					 	   0, 0, 1, 0, 0, 0 };	// Transformation matrix
	
	m_cvKalman = cvCreateKalman( 6, 3, 0 );
	m_matState 			= cvCreateMat( 6, 1, CV_32FC1 );
	m_matProcess_noise	= cvCreateMat( 6, 1, CV_32FC1 );
	m_matMeasurement 	= cvCreateMat( 3, 1, CV_32FC1 );

	memcpy( m_cvKalman->transition_matrix->data.fl, pT, sizeof(pT) );
	//memcpy( m_cvKalman->measurement_matrix->data.fl, pM, sizeof(pM) );
	cvSetIdentity( m_cvKalman->measurement_matrix, cvRealScalar(1) );
	cvSetIdentity( m_cvKalman->process_noise_cov, cvRealScalar(1e-5) );
	cvSetIdentity( m_cvKalman->measurement_noise_cov, cvRealScalar(1e-1) );
	cvSetIdentity( m_cvKalman->error_cov_post, cvRealScalar(.1) );
}

C2DOFKF::~C2DOFKF()
{
}

void C2DOFKF::init( Point pos, float theta )
{
	m_cvKalman->state_post->data.fl[0] = pos.x;
	m_cvKalman->state_post->data.fl[1] = pos.y;
	m_cvKalman->state_post->data.fl[2] = theta;
}

void C2DOFKF::update()
{
	const CvMat * matPredict;
	matPredict = cvKalmanPredict( m_cvKalman, 0 );
	m_ptPrePt.x = round( matPredict->data.fl[0] );  
	m_ptPrePt.y = round( matPredict->data.fl[1] );  
	m_fPreTheta = round( matPredict->data.fl[2] );  
}

void C2DOFKF::measure( Point pos, float theta )
{
	m_matMeasurement->data.fl[0] = pos.x;
	m_matMeasurement->data.fl[1] = pos.y;
	m_matMeasurement->data.fl[2] = theta;
	
	const CvMat * matCorrect;
	matCorrect = cvKalmanCorrect( m_cvKalman, m_matMeasurement );

	m_ptCorPt.x = round( matCorrect->data.fl[0] );
	m_ptCorPt.y = round( matCorrect->data.fl[1] );
	m_fCorTheta = round( matCorrect->data.fl[2] );
}

void C2DOFKF::output_pre( Point & pos, float & theta )
{
	pos = m_ptPrePt;
	theta = m_fPreTheta;
}

void C2DOFKF::output_correct( Point & pos, float & theta )
{
	pos = m_ptCorPt;
	theta = m_fCorTheta;
}
