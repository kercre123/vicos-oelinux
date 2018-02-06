/***************************************************************************//**
@copyright
Copyright (c) 2017 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include "../inc/MoveAlongPathPattern.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>



namespace MoveAlongPathPatternSpace
{

#define PI 3.1415926
STATUS gRobotStatus; 
bool grotatingMid2GoalFlag = false;


//------------------------------------------------------------------------------
/// @brief
///     Class constructor given the name of path pattern file
/// @param pathPatternFile
///     Name of path pattern file 
//------------------------------------------------------------------------------
MoveAlongPathPattern::MoveAlongPathPattern( string pathPatternFile )
{
	qCntPose = 0;
	qPathPatternFile = pathPatternFile;
	//printf( "In the construction function of MoveAlongPathPattern!\n" );
}



//------------------------------------------------------------------------------
/// @brief
///     Class deconstructor function 
//------------------------------------------------------------------------------
MoveAlongPathPattern::~MoveAlongPathPattern( )
{
	//if(NULL == qPoseArrary.poseArrary)
	//	delete[] qPoseArrary.poseArrary;

	//printf( "In the deconstrunction function of MoveAlongPathPattern!\n" );
}



//------------------------------------------------------------------------------
/// @brief
///     Load path pattern and convert the poses of the path to the coordination of FUSION pose
/// @param curPose
///     Fusion pose
//------------------------------------------------------------------------------
void MoveAlongPathPattern::loadPathPattern( Pose3D curPose )
{
	
	//FILE * readPathFile;
	//errno_t err = fopen_s(&readPathFile, qPathPatternFile.c_str(), "rb");
	FILE * readPathFile = fopen(qPathPatternFile.c_str(), "rb");

	if( readPathFile == NULL )
	//if (err != 0)
	{
		//printf( "Could not open the file of %s! But would run an example instead!\n", qPathPatternFile.c_str() );

		qRobotSpeed = { (float32_t)0.2, 0.0, 5.0 };
		qThresholdAngle = 4.5;
		qThresholdDist = (float32_t)0.15;
		 
		qPoseArray.push_back( { 0, 0, 0 } );
		qPoseArray.push_back( { 2, 0, 90 } );
		qPoseArray.push_back( { 2, 2, 180 } );
		qPoseArray.push_back( { 0, 2, 270 } );
		qPoseArray.push_back( { 0, 0, 0 } );
		convertPathPatternPoses2posecoord(  curPose );
	}

	else
	{
		qRobotSpeed = { (float32_t)0.2, 0.0, 5.0 };
		qThresholdAngle = (float32_t)4.5;
		qThresholdDist = (float32_t)0.15;
		//qPoseArrary.poseNum = 5;

		const int32_t size = 256;
		char linemsg[size];

		if( readPathFile == NULL )
		  return;

		string tmpString, numString;
		float32_t numValue;
		int32_t tmpPos, tmpLength;

		fgets( linemsg, size - 1, readPathFile );
		tmpString.append( linemsg );
		tmpPos = tmpString.find( " = ", 0 );
	    tmpLength = tmpString.length();
		numString = tmpString.substr(tmpPos + 3, tmpLength - tmpPos - 1);
		numValue = (float32_t)atof( numString.c_str() );
		qRobotSpeed.x = numValue;
		tmpString.erase();
		numString.empty();

		fgets( linemsg, size - 1, readPathFile );
		tmpString.append( linemsg );
		tmpPos = tmpString.find( " = ", 0 );
	    tmpLength = tmpString.length();
		numString = tmpString.substr(tmpPos + 3, tmpLength - tmpPos - 1);
		numValue = (float32_t)atof( numString.c_str() );
		qRobotSpeed.yaw = numValue;
		tmpString.erase();
		numString.empty();

		fgets( linemsg, size - 1, readPathFile );
		tmpString.append( linemsg );
		tmpPos = tmpString.find( " = ", 0 );
	    tmpLength = tmpString.length();
		numString = tmpString.substr(tmpPos + 3, tmpLength - tmpPos - 1);
		numValue = (float32_t)atof( numString.c_str() );
		qThresholdAngle = numValue;
		tmpString.erase();
		numString.empty();

		fgets( linemsg, size - 1, readPathFile );
		tmpString.append( linemsg );
		tmpPos = tmpString.find( " = ", 0 );
	    tmpLength = tmpString.length();
		numString = tmpString.substr(tmpPos + 3, tmpLength - tmpPos - 1);
		numValue = (float32_t)atof( numString.c_str() );
		qThresholdDist = numValue;
		tmpString.erase();
		numString.empty();


		fgets( linemsg, size - 1, readPathFile );
		tmpString.append( linemsg );
		tmpPos = tmpString.find( " = ", 0 );
	    tmpLength = tmpString.length();
		numString = tmpString.substr(tmpPos + 3, tmpLength - tmpPos - 1);
		numValue = (float32_t)atof( numString.c_str() );
		//qPoseArrary.poseNum = (int32_t)numValue;
		tmpString.erase();
		numString.empty();

		//qPoseArrary.poseArrary = new Pose3D[qPoseArrary.poseNum];
		int32_t cntPose = 0;
		while( NULL != fgets( linemsg, size - 1, readPathFile ) )
		{
		  linemsg[size - 1] = 0;  //???why
		  int32_t len = strlen( linemsg );
		  if( linemsg[0] == '%' || len < 2 )
			 // comment line or empty string
			 continue;

		  // deal with UNIX or Windows Line terminator
		  if( linemsg[len - 1] == '\n' || linemsg[len - 1] == '\r' )
		  {
			 linemsg[len - 1] = '\0';
		  }
		  if( linemsg[len - 2] == '\n' || linemsg[len - 2] == '\r' )
		  {
			 linemsg[len - 2] = '\0';
		  }

		  string tmpString(linemsg);
		  int32_t startPos = 0, cnt = 0;
		  bool success2savePose = false;
		  float32_t poseV[3];
		  while(1)
		  {
			  int32_t endPos = tmpString.find( ",", startPos );
			  int32_t numChar = endPos - startPos;
			  string numString = tmpString.substr(startPos, numChar);

			  float32_t numValue = (float32_t)atof( numString.c_str() );
			  //printf( "%d %d %s %f\n", startPos, endPos, numString.c_str(), numValue );
			  poseV[cnt++] = numValue;

			  startPos = endPos + 1;
			  if(cnt == 3)
			  {
				  success2savePose = true;
				  break;
			  }
		  }

		  if( success2savePose )
		  {
			  //qPoseArrary.poseArrary[cntPose].x = poseV[0];
			  //qPoseArrary.poseArrary[cntPose].y = poseV[1];
			  //qPoseArrary.poseArrary[cntPose].yaw = poseV[2];
			  cntPose++;
		  }
		}

		//fclose( readPathFile );
		//printf("Finish reading the path pattern file!\n");

		convertPathPatternPoses2posecoord(  curPose );
	}
}



//------------------------------------------------------------------------------
/// @brief
///     Check the orientation of the pose and make it in the predefined range
/// @param pose
///     Pose whose orientation will be checked and might be modified
//------------------------------------------------------------------------------
void checkPoseOrientationRange( Pose3D &pose )
{
	int32_t k = int32_t(pose.yaw / 360);
	if( k != 0 )
		pose.yaw = pose.yaw - k * 360;

	if( pose.yaw < 0.0 )
		pose.yaw += 360;
}


//------------------------------------------------------------------------------
/// @brief
///     Convert the poses of the pat to the coordination of the fusion pose
/// @param curPose
///     Fusion pose
//------------------------------------------------------------------------------
void MoveAlongPathPattern::convertPathPatternPoses2posecoord( Pose3D curPose )
{
	if( 0 == qPoseArray.size() )
		return;

	// need to check the angle to make it in the range, else would be harmful
	qCurPose = curPose;
	Pose3D deltaPose = qCurPose - qPoseArray[0];

	for( size_t cnt = 0; cnt < qPoseArray.size(); cnt++)
	{
		qPoseArray[cnt] = qPoseArray[cnt] + deltaPose;
		checkPoseOrientationRange( qPoseArray[cnt] );
	}

	//printf("Finish alignment between curPose and the path pattern!\n");
}



//------------------------------------------------------------------------------
/// @brief
///     Determine the rotation orientation from source to destination
/// @param srcAngle
///     Angle of rotating from
/// @param dstAngle
///     Angle of rotating to
/// @param rotateAngle
///     Pointer to the rotation angle
//------------------------------------------------------------------------------
void compuateRotateAngle( float32_t srcAngle, float32_t dstAngle, float32_t * rotateAngle )
{
	float32_t deltaAngle = dstAngle - srcAngle;

	if( deltaAngle >= 0.0 )
	{
		if( deltaAngle < fabs( -360.0 + deltaAngle ) )
		{
			rotateAngle[0] = deltaAngle;
			rotateAngle[1] = (float32_t)(-180.0 + deltaAngle);
		}
		else
		{
			rotateAngle[0] = (float32_t)(-360.0 + deltaAngle);
			rotateAngle[1] = (float32_t)(180.0 + rotateAngle[0]);
		}
	}
	else
	{
		if( fabs( deltaAngle ) < fabs( 360 + deltaAngle ) )
		{
			rotateAngle[0] = deltaAngle;
			rotateAngle[1] = (float32_t)(180.0 + deltaAngle);
		}
		else
		{
			rotateAngle[0] = 360 + deltaAngle;
			rotateAngle[1] = (float32_t)(-180.0 + rotateAngle[0]);
		}
	}
}


//------------------------------------------------------------------------------
/// @brief
///     Determine whether the current pose reaches the set goal or not
//------------------------------------------------------------------------------
bool MoveAlongPathPattern::isReachGoal(  )
{
	bool result = true;

	float32_t distSquare = getDistSquare( qCurGoal, qCurPose );
	float32_t diffAngle = getDiffAngle( qCurGoal, qCurPose );

	//printf( "distSquare = %f\t_thresholdDist^2 = %f\tdiffAngle = %f\t_thresholdAngle = %f\n", distSquare, qThresholdDist, diffAngle, qThresholdAngle );
	if( ( (distSquare > qThresholdDist * qThresholdDist) && !grotatingMid2GoalFlag ) || (diffAngle > qThresholdAngle) )
	{
		result = false;
	}

	return result;
}


//------------------------------------------------------------------------------
/// @brief
///     Shortest path comoputation based on the smallest rotation orientation
/// @param angle2midPose
///     Angle rotating from current pose to middle pose
/// @param angle2goalPose
///     Angle of rotating from middle pose to goal pose
/// @param isMoveFoward
///     Positive for moving forward and negative for moving backward
//------------------------------------------------------------------------------
void MoveAlongPathPattern::computeLeastRotationPath( float32_t &angle2midPose, float32_t &angle2goalPose, bool &isMoveFoward )
{
	// Compute the orientation between curPose to curGoal
	double angle4LinePose2Goal;
	double deltaX = qCurGoal.x - qCurPose.x;
	double deltaY = qCurGoal.y - qCurPose.y;
	angle4LinePose2Goal = atan2( deltaY, deltaX );
	if( angle4LinePose2Goal >= 0 )
	{
		angle4LinePose2Goal = angle4LinePose2Goal / PI * 180.0;
	}
	else
	{
		angle4LinePose2Goal = 360.0 + angle4LinePose2Goal / PI * 180.0;
	}

	// Compute paths from curPose to curGoal and select one based on the least rotation
	// 1. Compute the smallest angle from curPose to angle4LinePose2Goal
	// 2. Compute the smallest angle from angle4LinePose2Goal to the curGoal
	// 3. Compare the angle rotated in total and select the smallest one
	float32_t angle4curPose2midPose[2], angle4midPose2goalPose[2];
	compuateRotateAngle( qCurPose.yaw, (float32_t)angle4LinePose2Goal, angle4curPose2midPose );
	compuateRotateAngle( (float32_t)angle4LinePose2Goal, qCurGoal.yaw, angle4midPose2goalPose );

	float32_t sumAngle1 = fabs( angle4curPose2midPose[0] ) + fabs( angle4midPose2goalPose[0] );
	float32_t sumAngle2 = fabs( angle4curPose2midPose[1] ) + fabs( angle4midPose2goalPose[1] );
	if(sumAngle1 < sumAngle2)
	{
		angle2midPose = angle4curPose2midPose[0];
		angle2goalPose = angle4midPose2goalPose[0];
		isMoveFoward = true;
	}
	else
	{
		angle2midPose = angle4curPose2midPose[1];
		angle2goalPose = angle4midPose2goalPose[1];
		isMoveFoward = false;
	}
}


//------------------------------------------------------------------------------
/// @brief
///     Computation the velocity
//------------------------------------------------------------------------------
void MoveAlongPathPattern::computeVelocity( )
{
	qCmdVel = {0.0, 0.0, 0.0};

	// Compute paths from curPose to curGoal and select one based on the least rotation
	float32_t angle4curPose2midPose, angle4midPose2goalPose;
	bool isMoveFoward;
	computeLeastRotationPath( angle4curPose2midPose, angle4midPose2goalPose, isMoveFoward );

	// Control the robot to move in this order
	// 1. rotate to the line of curPose & curGoal
	// 2. move to the location of curGoal
	// 3. rotate to the orientation of curGoal
	static bool rotatingMid2GoalFlag = false;
	if( gRobotStatus == UPDATE_GOAL_STATUS )
		rotatingMid2GoalFlag = false;

	grotatingMid2GoalFlag = rotatingMid2GoalFlag;

	//printf("angle4curPose2midPose = %f qThresholdAngle = %f, rotatingMid2GoalFlag = %d\n", angle4curPose2midPose, qThresholdAngle, rotatingMid2GoalFlag);
	//printf("getDistSquare( qCurGoal, qCurPose ) = %f qThresholdDist^2 = %f\n", getDistSquare( qCurGoal, qCurPose ), qThresholdDist * qThresholdDist);
	//printf("angle4midPose2goalPose = %f qThresholdAngle = %f\n", angle4midPose2goalPose, qThresholdAngle);

	if( (fabs(angle4curPose2midPose) > qThresholdAngle) && (!rotatingMid2GoalFlag) )
	{
		qCmdVel.yaw = qRobotSpeed.yaw;
		if( angle4curPose2midPose < 0.0 )
			qCmdVel.yaw *= -1.0;
	}
	else if( (getDistSquare( qCurGoal, qCurPose ) > qThresholdDist * qThresholdDist) && (!rotatingMid2GoalFlag) )
	{
		qCmdVel.x = qRobotSpeed.x;
		if( !isMoveFoward )
			qCmdVel.x *= -1.0;
	}
	//else if( fabs(angle4midPose2goalPose) > qThresholdAngle )
	else if( getDiffAngle( qCurGoal, qCurPose ) > qThresholdAngle )
	{
		rotatingMid2GoalFlag = true;
		float32_t rotateAngle[2];
		compuateRotateAngle( qCurPose.yaw, qCurGoal.yaw, rotateAngle );
		//printf("rotateAngle = %f \t %f\n", rotateAngle[0], rotateAngle[1]);

		/*if( fabs(rotateAngle[0]) > fabs(rotateAngle[1]) )
			angle4midPose2goalPose = rotateAngle[1];
		else
			angle4midPose2goalPose = rotateAngle[0];*/

		angle4midPose2goalPose = rotateAngle[0];

		qCmdVel.yaw = qRobotSpeed.yaw;
		if( angle4midPose2goalPose < 0.0 )
			qCmdVel.yaw *= -1.0;
	}
	else
	{
		 //printf("In the function of computeVelocity, should not call here!\n");
	}
}


//------------------------------------------------------------------------------
/// @brief
///     Interface of the path pattern part
/// @param curPose
///     Pose from fusion
/// @param cmdVel
///     Velocity determined for controlling robot
//------------------------------------------------------------------------------
bool MoveAlongPathPattern::runPathPattern( Pose3D curPose, Velocity &cmdVel )
{
	bool isFinishFlag = false;
	//static int32_t cntPose = 0; 
	if( 0 == qPoseArray.size() || qCntPose >= qPoseArray.size() )
	{
		isFinishFlag = true;
		return isFinishFlag;
	}
	 
	if( isReachGoal(  ) )
	{
		qCntPose++;
		if( qCntPose >= qPoseArray.size() )
		{
			gRobotStatus = FINISH_STATUS;
		}
		else
		{
			gRobotStatus = UPDATE_GOAL_STATUS;
		}
	}
	else
	{
		gRobotStatus = MOVE_ROBOT_STATUS;
	}
	
   if( gRobotStatus != FINISH_STATUS )
   {
      qCurGoal = qPoseArray[qCntPose];
   }	
	qCurPose = curPose;


	switch( gRobotStatus )
	{
	case UPDATE_GOAL_STATUS:
		qCurGoal = qPoseArray[ qCntPose ];
		//printf( "Set new goal as (x = %f\ty = %f\tyaw = %f\n)", qCurGoal.x, qCurGoal.y, qCurGoal.yaw );

	case MOVE_ROBOT_STATUS:
		computeVelocity( );
		cmdVel = qCmdVel;
		printf( "curPose = (%f %f %f), curGoal = (%f %f %f), curSpeed = (%f %f %f)\n", qCurPose.x, qCurPose.y, qCurPose.yaw, qCurGoal.x, qCurGoal.y, qCurGoal.yaw, qCmdVel.x, qCmdVel.y, qCmdVel.yaw );
		break;

	case FINISH_STATUS:
		isFinishFlag = true;
		//printf( "Finish the given path pattern!\n" );
		break;

	default:
		//printf( "Error and should not be run to the default case!\n" );
		break;
	}

	return isFinishFlag;
}

} // namespace MoveAlongPathPatternSpace
