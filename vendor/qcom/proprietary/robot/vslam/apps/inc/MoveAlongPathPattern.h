/******************************************************************************
@copyright
Copyright (c) 2017 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef _MOVE_ALONG_PATH_PATTER_
#define _MOVE_ALONG_PATH_PATTER_


//==============================================================================
// Defines
//==============================================================================


//==============================================================================
// Includes
//==============================================================================
#include <iostream>
#include <stdio.h>
#include "mv.h" 
#include <vector>

using namespace std;

namespace MoveAlongPathPatternSpace
{


/**------------------------------------------------------------------------------
@brief
Status for robot movement during moving along the given path pattern 
------------------------------------------------------------------------------**/
enum STATUS { UPDATE_GOAL_STATUS, MOVE_ROBOT_STATUS, FINISH_STATUS };

 
/**------------------------------------------------------------------------------
@brief
Pose information including the location of (x, y) in meter and the orientation of yaw in degree [0, 360)
------------------------------------------------------------------------------**/
struct Pose3D
{
	float32_t x;    // the location of axis x
	float32_t y;    // the location of axis y
	float32_t yaw;  // the orientation of yaw
};



/**------------------------------------------------------------------------------
@brief
Velocity of the robot including linarly speed x m/s and angualar speed degree/s
------------------------------------------------------------------------------**/
struct Velocity
{
	float32_t x;    // speed of x direction in m/s
	float32_t y;    // speed of y direction in m/s
	float32_t yaw;  // speed of yaw direction in rad/s
};


/**------------------------------------------------------------------------------
@brief
Pathpattern descriptor for the given path including poses in order and its number
------------------------------------------------------------------------------**/
//struct pathPattern
//{
//	Pose3D * poseArrary = NULL;
//	int32_t poseNum = 0;
//};

typedef std::vector<Pose3D> pathPattern;

 
/**------------------------------------------------------------------------------
@brief
Substraction between two poses: pose1 - pose2
@param pose1
	Pose for subtractor
@param pose2
	Pose for minuend
------------------------------------------------------------------------------**/
inline Pose3D operator - (Pose3D& pose1, Pose3D& pose2)
{
	Pose3D outputPose;

	outputPose.x = pose1.x - pose2.x;
	outputPose.y = pose1.y - pose2.y;
	outputPose.yaw = pose1.yaw - pose2.yaw;

	// would be notice the angle

	return outputPose;
}


/**------------------------------------------------------------------------------
@brief
Addition between two poses: pose1 + pose2
@param pose1
	Pose for one addend
@param pose2
	Pose for another addend
------------------------------------------------------------------------------**/
inline Pose3D operator + (Pose3D& pose1, Pose3D& pose2)
{
	Pose3D outputPose;

	outputPose.x = pose1.x + pose2.x;
	outputPose.y = pose1.y + pose2.y;
	outputPose.yaw = pose1.yaw + pose2.yaw;

	// would be notice the angle

	return outputPose;
}

 
/**------------------------------------------------------------------------------
@brief
Computer the orientation different between two poses
@param pose1
	Pose whose orientation is taken as subtractor
@param pose2
	Pose whose orientation is taken as minuend
------------------------------------------------------------------------------**/
inline float32_t getDiffAngle( Pose3D nextGoal, Pose3D curPose )
{
	float32_t tmpDiffAngle = nextGoal.yaw - curPose.yaw;
	float32_t diffAngle = tmpDiffAngle > 0 ? tmpDiffAngle : -1 * tmpDiffAngle;

	if( diffAngle > 180 )
		diffAngle = 360 - diffAngle;

	return diffAngle;
}


/**------------------------------------------------------------------------------
@brief
Computer the translation different between two poses
@param pose1
	Pose whose location is taken as subtractor
@param pose2
	Pose whose location is taken as minuend
------------------------------------------------------------------------------**/
inline float32_t getDistSquare( Pose3D nextGoal, Pose3D curPose )
{
	float32_t distSquare = (nextGoal.x - curPose.x) * (nextGoal.x - curPose.x)
			+ (nextGoal.y - curPose.y) * (nextGoal.y - curPose.y);

	return distSquare;
}


class MoveAlongPathPattern
{
public:
	MoveAlongPathPattern() { qCntPose = 0; }
	MoveAlongPathPattern( string pathPatternFile );
	~MoveAlongPathPattern( );

	void loadPathPattern( Pose3D curPose );
	void convertPathPatternPoses2posecoord( Pose3D curPose );
	bool runPathPattern( Pose3D curPose, Velocity &cmdVel );
	 
	void computeVelocity( );
	void computeLeastRotationPath( float32_t &angle2midPose, float32_t &angle2goalPose, bool &isMoveFoward );

	bool isReachGoal(  );
	 
public:
	string qPathPatternFile;    // file which contains the poses of path pattern, the angle/linear speed, the threshold of angle/distance between poses
	Velocity qRobotSpeed;       // robot speed during movement
	float32_t qThresholdAngle;  // accept orientation error during movement
	float32_t qThresholdDist;   // accept translation error during movement

	pathPattern qPoseArray;    // variable for store the poses of the given path information
	size_t qCntPose;               // the index of poses in the path beening processed
	Pose3D qCurPose;            // pose for the current robot
	Pose3D qCurGoal;            // pose for the robot to reach

	Velocity qCmdVel;           // velocity of the robot for the current movement
};

} // namespace MoveAlongPathPatternSpace

#endif //_MOVE_ALONG_PATH_PATTER_
