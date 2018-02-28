/*****************************************************************************
@copyright
Copyright (c) 2017 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef  __MAP_FOCUSER_H__
#define __MAP_FOCUSER_H__

#include "mvVSLAM.h"
#include <list>
#include <vector>
#include <set>
#include <mutex>

namespace cv
{
   class Mat;
}


class MapFocuser
{	
   friend void DetermineVisualizationSize(MapFocuser & mapFocuser, cv::Mat &);
public:
	struct KeyframeDistance
	{
		float distance;
		int id;
	};

	struct MapRect
	{
		float left;
		float right;
		float bottom;
		float top;
	};

	MapFocuser();
	~MapFocuser();
	void SetCrossCalibrationMatrix(const float* matrixData);
	void SetSpatialInWorldMatrix(const float* matrixData);
	bool NeedSpatialInWorld() const;
	bool TransformationReady() const;
	void AddUnbiasedPose(mvVSLAMTrackingPose& pose);
	void AddPoseInWorld(mvVSLAMTrackingPose& pose);
	void AddScalessPose(mvVSLAMTrackingPose& pose);
	void UpdateKeyframeState(MV_ActiveKeyframe* keyframes, int nSize, bool scaleFlag);
	void AddCleanArea(float x1, float y1, float x2, float y2);
	bool InCleanArea(float x, float y);
	void ClearTrajectory();
	std::set<int> GetKFsIdToLoad();
	std::set<int> GetKFsIdToDeactivate();
	std::set<int> GetKFsIdToRemove();

protected:
	void TransformPose(const float pose[3][4], const float transformer[3][4], float newPose[3][4]);
	bool AnalyzeDistribution();
	bool CheckColinearity(float x0, float y0, float x1, float y1, float x2, float y2);

	float m_xMinKF;                                         /*Lei: minimum x value of keyframes*/
	float m_xMaxKF;                                         /*Lei: maximum x value of keyframes*/
	float m_yMinKF;                                         /*Lei: minimum y value of keyframes*/
	float m_yMaxKF;                                         /*Lei: maximum y value of keyframes*/

	float m_baselinkInVSLAM[3][4];                          /*Lei: across calibration matrix*/
	float m_spatialInWorld[3][4];                           /*Lei: transformation from VSLAM reference to world coordinate system*/
	bool m_flagBVReady;                                     /*Lei: indicator of across calibration data ready*/
	bool m_flagSWReady;                                     /*Lei: indicator of spatial-in-world data ready*/
	bool m_flagScale;                                       /*Lei: indicator of scale-in-world/scaless*/

	unsigned int m_sizeHistory;                             /*Lei: control number of poses*/
	std::list<mvVSLAMTrackingPose> m_poseHistory;           /*Lei: recorded poses for prediction and visualization*/
	std::list<mvVSLAMTrackingPose> m_scalessPoseHistory;    /*Lei: recorded scaless poses for prediction and visualization*/
	std::vector<MV_ActiveKeyframe> m_activeKeyframes;       /*Lei: whole set of active keyframes*/

	int m_smoothWindow;                                     /*Lei: window size to average past positions*/
	float m_kernalRadius;
	std::set<int> m_kernalKFs;
	std::set<int> m_ambientKFs;
	std::set<int> m_removedKFs;                             /*Lei: keyframes that have been removed*/
	std::set<int> m_farawayKFs;                             /*Lei: keyframes that have been removed*/
	std::list<MapRect> m_cleanAreas;                        /*Lei: keyframes in this area can be removed permanently*/

	int m_maxActiveKFs;

   std::mutex  historyMutex;

};


#endif