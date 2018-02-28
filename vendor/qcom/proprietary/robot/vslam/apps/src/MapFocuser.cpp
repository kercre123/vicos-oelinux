/*****************************************************************************
@copyright
Copyright (c) 2017 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include "MapFocuser.h"
#include <assert.h>
#include <string.h>

bool compare_distance( const MapFocuser::KeyframeDistance& first, const MapFocuser::KeyframeDistance& second )
{
   return first.distance < second.distance;
}

MapFocuser::MapFocuser()
{
   m_flagBVReady = false;
   m_flagSWReady = false;
	m_flagScale = false;
   m_sizeHistory = 200;

   m_smoothWindow = 5;
   m_kernalRadius = 2.5f;
   m_maxActiveKFs = 30;

   m_xMinKF = m_yMinKF = -2.0f;
   m_xMaxKF = m_yMaxKF = 2.0f;

}


MapFocuser::~MapFocuser()
{

}


void MapFocuser::AddUnbiasedPose(mvVSLAMTrackingPose& pose)
{
	if (!m_flagBVReady || !m_flagSWReady) /*Lei: only works in world coordinate system*/
	{
		m_poseHistory.clear();
		return;
	}

	float baselinkInSpatial[3][4];
	mvVSLAMTrackingPose	baselinkInWorld;
	baselinkInWorld.poseQuality = pose.poseQuality;
	TransformPose(m_baselinkInVSLAM, pose.pose.matrix, baselinkInSpatial);
	TransformPose(baselinkInSpatial, m_spatialInWorld, baselinkInWorld.pose.matrix);

	historyMutex.lock();
	m_poseHistory.push_front(baselinkInWorld);	
	if (m_poseHistory.size() > m_sizeHistory)
		m_poseHistory.resize(m_sizeHistory);
	historyMutex.unlock();
}


void MapFocuser::AddScalessPose(mvVSLAMTrackingPose& pose)
{
	historyMutex.lock();
	m_scalessPoseHistory.push_front(pose);
	if (m_scalessPoseHistory.size() > m_sizeHistory)
		m_scalessPoseHistory.resize(m_sizeHistory);
	historyMutex.unlock();
}


void MapFocuser::AddPoseInWorld( mvVSLAMTrackingPose& pose )
{
   historyMutex.lock();
   m_poseHistory.push_front( pose );
   if( m_poseHistory.size() > m_sizeHistory )
      m_poseHistory.resize( m_sizeHistory );
   historyMutex.unlock();
}


void MapFocuser::UpdateKeyframeState(MV_ActiveKeyframe* keyframes, int nSize, bool scaleFlag)
{
	if (scaleFlag != m_flagScale)
	{
		m_xMinKF = m_yMinKF = -2.0f;
		m_xMaxKF = m_yMaxKF = 2.0f;
		m_poseHistory.clear();
		m_scalessPoseHistory.clear();
	}
	m_flagScale = scaleFlag;

	if (m_flagScale)
	{
   m_activeKeyframes.clear();
   m_activeKeyframes.reserve( nSize );
   for( int i = 0; i < nSize; ++i )
   {
      float baselinkInSpatial[3][4];
      MV_ActiveKeyframe baselinkInWorld;
      baselinkInWorld.id = keyframes[i].id;
      TransformPose( m_baselinkInVSLAM, keyframes[i].pose.matrix, baselinkInSpatial );
      TransformPose( baselinkInSpatial, m_spatialInWorld, baselinkInWorld.pose.matrix );
      m_activeKeyframes.push_back( baselinkInWorld );
      m_xMinKF = m_xMinKF < baselinkInWorld.pose.matrix[0][3] ? m_xMinKF : baselinkInWorld.pose.matrix[0][3];
      m_xMaxKF = m_xMaxKF > baselinkInWorld.pose.matrix[0][3] ? m_xMaxKF : baselinkInWorld.pose.matrix[0][3];
      m_yMinKF = m_yMinKF < baselinkInWorld.pose.matrix[1][3] ? m_yMinKF : baselinkInWorld.pose.matrix[1][3];
      m_yMaxKF = m_yMaxKF > baselinkInWorld.pose.matrix[1][3] ? m_yMaxKF : baselinkInWorld.pose.matrix[1][3];
   }
		for (std::list<MapRect>::iterator i = m_cleanAreas.begin(); i != m_cleanAreas.end(); ++i)
		{
			m_xMinKF = m_xMinKF < i->left ? m_xMinKF : i->left;
			m_xMaxKF = m_xMaxKF > i->right ? m_xMaxKF : i->right;
			m_yMinKF = m_yMinKF < i->bottom ? m_yMinKF : i->bottom;
			m_yMaxKF = m_yMaxKF > i->top ? m_yMaxKF : i->top;
		}		
	}
	else
	{
		m_activeKeyframes.clear();
		m_activeKeyframes.reserve(nSize);
		for (int i = 0; i < nSize; ++i)
		{
			m_activeKeyframes.push_back(keyframes[i]);
			m_xMinKF = m_xMinKF < keyframes[i].pose.matrix[0][3] ? m_xMinKF : keyframes[i].pose.matrix[0][3];
			m_xMaxKF = m_xMaxKF > keyframes[i].pose.matrix[0][3] ? m_xMaxKF : keyframes[i].pose.matrix[0][3];
			m_yMinKF = m_yMinKF < keyframes[i].pose.matrix[1][3] ? m_yMinKF : keyframes[i].pose.matrix[1][3];
			m_yMaxKF = m_yMaxKF > keyframes[i].pose.matrix[1][3] ? m_yMaxKF : keyframes[i].pose.matrix[1][3];
		}
	}

   AnalyzeDistribution();
}


bool MapFocuser::AnalyzeDistribution()
{
   historyMutex.lock();
    if ((int)(m_poseHistory.size()) < m_smoothWindow && m_flagScale)
   {
      historyMutex.unlock();
      return false;
   }
	if ((int)(m_scalessPoseHistory.size()) < m_smoothWindow && !m_flagScale)
	{
		historyMutex.unlock();
		return false;
	}

	if (m_flagScale)
	{
		int i;
		float meanX, meanY;
		std::list<mvVSLAMTrackingPose>::iterator onePose;
		for (onePose = m_poseHistory.begin(), i = 0, meanX = 0, meanY = 0; i < m_smoothWindow; ++i)
		{
			meanX += onePose->pose.matrix[0][3];
			meanY += onePose->pose.matrix[1][3];
		}		

		meanX /= m_smoothWindow;
		meanY /= m_smoothWindow;

   std::list<KeyframeDistance> kfDistance;
   for( unsigned int j = 0; j < m_activeKeyframes.size(); ++j )
   {
      if( m_removedKFs.find( m_activeKeyframes[j].id ) != m_removedKFs.end() )
         continue;
      KeyframeDistance newOne;
      newOne.distance = (m_activeKeyframes[j].pose.matrix[0][3] - meanX)*(m_activeKeyframes[j].pose.matrix[0][3] - meanX) + (m_activeKeyframes[j].pose.matrix[1][3] - meanY)*(m_activeKeyframes[j].pose.matrix[1][3] - meanY);
      newOne.id = m_activeKeyframes[j].id;
      kfDistance.push_back( newOne );
   }
   kfDistance.sort( compare_distance );

   m_kernalKFs.clear();
   m_ambientKFs.clear();
   std::list<KeyframeDistance>::iterator oneDistance;
   for( oneDistance = kfDistance.begin(), i = 0; oneDistance != kfDistance.end() && i < m_maxActiveKFs; oneDistance++, i++ )
   {
      if( oneDistance->distance < m_kernalRadius*m_kernalRadius )
      {
         m_kernalKFs.insert( oneDistance->id );
      }
      else
         break;
   }
   for( ; oneDistance != kfDistance.end() && i < m_maxActiveKFs; oneDistance++, i++ )
      m_ambientKFs.insert( oneDistance->id );

   /*Lei: to determine keyframes to remove or deactivate*/
   m_farawayKFs.clear();
   for( unsigned int j = 0; j < m_activeKeyframes.size(); ++j )
   {
      if( m_removedKFs.find( m_activeKeyframes[j].id ) != m_removedKFs.end() )
         continue;
      int id = m_activeKeyframes[j].id;
      if( m_ambientKFs.find( id ) == m_ambientKFs.end() && m_kernalKFs.find( id ) == m_kernalKFs.end() )
      {
         if( InCleanArea( m_activeKeyframes[j].pose.matrix[0][3], m_activeKeyframes[j].pose.matrix[1][3] ) )
            m_removedKFs.insert( id );
         else
            m_farawayKFs.insert( id );
      }
   }
	}
	historyMutex.unlock();

   return true;
}


bool MapFocuser::CheckColinearity( float x0, float y0, float x1, float y1, float x2, float y2 )
{
   return false;
}


void MapFocuser::SetCrossCalibrationMatrix( const float* matrixData )
{
   const int length = 4;
   for( int i = 0; i < 3; i++ )
   {
      memcpy( m_baselinkInVSLAM[i], matrixData + i * length, sizeof( float ) * length );
   }
   m_flagBVReady = true;
}


void MapFocuser::SetSpatialInWorldMatrix( const float* matrixData )
{
   const int length = 4;
   for( int i = 0; i < 3; i++ )
   {
      memcpy( m_spatialInWorld[i], matrixData + i * length, sizeof( float ) * length );
   }
   m_flagSWReady = true;
}


bool MapFocuser::NeedSpatialInWorld() const
{
   return !m_flagSWReady;
}


bool MapFocuser::TransformationReady() const
{
   return m_flagBVReady && m_flagSWReady;
}


void MapFocuser::AddCleanArea( float x1, float y1, float x2, float y2 )
{
   MapRect newArea;
   newArea.left = x1;
   newArea.right = x2;
   newArea.bottom = y1;
   newArea.top = y2;
   m_cleanAreas.push_back( newArea );
}


bool MapFocuser::InCleanArea( float x, float y )
{
   for( std::list<MapRect>::iterator i = m_cleanAreas.begin(); i != m_cleanAreas.end(); i++ )
   {
      if( x > i->left && x<i->right && y>i->bottom && y < i->top )
         return true;
   }

   return false;
}


void MapFocuser::TransformPose( const float pose[3][4], const float transformer[3][4], float newPose[3][4] )
{
   float sum;
   for( int i = 0; i < 3; ++i )
   {
      for( int j = 0; j < 3; ++j )
      {
         sum = 0.0f;
         for( int k = 0; k < 3; ++k )
            sum += transformer[i][k] * pose[k][j];
         newPose[i][j] = sum;
      }
      sum = 0.0f;
      for( int k = 0; k < 3; ++k )
         sum += transformer[i][k] * pose[k][3];
      newPose[i][3] = sum + transformer[i][3];
   }
}


void MapFocuser::ClearTrajectory()
{
   m_poseHistory.clear();
}


std::set<int> MapFocuser::GetKFsIdToLoad()
{
   return m_kernalKFs;
}


std::set<int> MapFocuser::GetKFsIdToDeactivate()
{
   return m_farawayKFs;
}


std::set<int> MapFocuser::GetKFsIdToRemove()
{
   return m_removedKFs;
}
