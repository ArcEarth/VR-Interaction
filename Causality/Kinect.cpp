#include <Kinect.h>
#include "Kinect.h"
using namespace Platform::Devices;

JointType Kinect::JointsParent[JointType_Count]{
	JointType_SpineBase, // JointType_SpineBase, Root
	JointType_SpineBase, // JointType_SpineMid
	JointType_SpineShoulder, // JointType_Neck
	JointType_Neck,  //JointType_Head
	JointType_SpineShoulder,  //JointType_ShoulderLeft
	JointType_ShoulderLeft, // JointType_ElbowLeft
	JointType_ElbowLeft, // JointType_WristLeft
	JointType_WristLeft,  // JointType_HandLeft
	JointType_SpineShoulder,  // JointType_ShoulderRight
	JointType_ShoulderRight,  // JointType_ElbowRight
	JointType_ElbowRight,  // JointType_WristRight
	JointType_WristRight,  // JointType_HandRight
	JointType_SpineBase,  // JointType_HipLeft
	JointType_HipLeft,  // JointType_KneeLeft
	JointType_KneeLeft,  // JointType_AnkleLeft
	JointType_AnkleLeft,  // JointType_FootLeft
	JointType_SpineBase,  // JointType_HipRight
	JointType_HipRight,  // JointType_KneeRight
	JointType_KneeRight,  // JointType_AnkleRight
	JointType_AnkleRight,  // JointType_FootRight
	JointType_SpineMid,  // JointType_SpineShoulder
	JointType_HandLeft, // JointType_HandTipLeft
	JointType_HandLeft,  // JointType_ThumbLeft
	JointType_HandRight,  // JointType_HandTipRight
	JointType_HandRight,  //JointType_ThumbRight
}