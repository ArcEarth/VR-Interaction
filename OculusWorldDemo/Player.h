/************************************************************************************

Filename    :   Player.h
Content     :   Avatar movement and collision detection
Created     :   October 4, 2012

Copyright   :   Copyright 2012 Oculus, Inc. All Rights reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.


*************************************************************************************/

#ifndef OVR_WorldDemo_Player_h
#define OVR_WorldDemo_Player_h

#include "OVR_Kernel.h"
#include "Kernel/OVR_KeyCodes.h"
#include "../CommonSrc/Render/Render_Device.h"
#include <boost\operators.hpp>
#include "Util\Filter.h"

using namespace OVR;
using namespace OVR::Render;
//-------------------------------------------------------------------------------------
// The RHS coordinate system is assumed.  
const Vector3f	RightVector(1.0f, 0.0f, 0.0f);
const Vector3f	UpVector(0.0f, 1.0f, 0.0f);
const Vector3f	ForwardVector(0.0f, 0.0f, -1.0f); // -1 because HMD looks along -Z at identity orientation

const float		YawInitial	= 0.0f;
const float		Sensitivity	= 0.3f; // low sensitivity to ease people into it gently.
const float		MoveSpeed	= 3.0f; // m/s

// These are used for collision detection
const float		RailHeight	= 0.8f;


namespace VectorMath{
	template <class T>
	class Angle
		: boost::addable< Angle<T>          // point + point
		, boost::subtractable< Angle<T>     // point - point
		, boost::dividable2< Angle<T>, T    // point / T
		, boost::multipliable2< Angle<T>, T // point * T, T * point
		> > > >
	{
	private:
		T	val;
	public:
		Angle()
		{
			val = 0;
		}

		explicit Angle(T rhs)
		{
			val = rhs;
			FixRange();
		}

		Angle& operator += (const Angle& rhs)
		{
			val += rhs.val;
			FastFixRange();
			return *this;
		}
		Angle& operator -= (const Angle& rhs)
		{
			val -= rhs.val;
			FastFixRange();
			return *this;
		}
		Angle& operator *= (T rhs){
			val *= rhs;
			FixRange();
			return *this;
		}
		Angle& operator /= (T rhs){
			val /= rhs;
			FixRange();
			return *this;
		}

		operator T() const
		{
			return val;
		}

		// Fixes the angle range to [-Pi,Pi], but assumes no more than 2Pi away on either side 
		inline void FastFixRange()
		{
			if (val < -((T) MATH_DOUBLE_PI))
				val += ((T) MATH_DOUBLE_TWOPI);
			else if (val >((T) MATH_DOUBLE_PI))
				val -= ((T) MATH_DOUBLE_TWOPI);
		}

		// Fixes the angle range to [-Pi,Pi] for any given range, but slower then the fast method
		inline void FixRange()
		{
			// do nothing if the value is already in the correct range, since fmod call is expensive
			if (val >= -((T) MATH_DOUBLE_PI) && val <= ((T) MATH_DOUBLE_PI))
				return;
			val = fmod(val, ((T) MATH_DOUBLE_TWOPI));
			if (val < -((T) MATH_DOUBLE_PI))
				val += ((T) MATH_DOUBLE_TWOPI);
			else if (val >((T) MATH_DOUBLE_PI))
				val -= ((T) MATH_DOUBLE_TWOPI);
		}
	};
}

//-------------------------------------------------------------------------------------
// ***** Player

// Player class describes position and movement state of the player in the 3D world.
class Player
{
public:

	float				UserEyeHeight;

	// Where the avatar coordinate system (and body) is positioned and oriented in the virtual world
    // Modified by gamepad/mouse input
	Vector3f			BodyPos;
	Anglef				BodyYaw;	// The direction of movement

	Anglef				AdditionalYaw;	// Additional yaw term apply to both body yaw and head orientation

    // Where the player head is positioned and oriented in the real world
    Posef				HeadPose;
    // Where the avatar head is positioned and oriented in the virtual world

	Vector3f            GetHeadPosistion();
    Quatf               GetBodyOrientation(bool baseOnly = false);
	void				SyncBodyYawToHeadYaw();
	void				HandleBodyOrientationSensorChanged(const Quatf & q);

    // Returns virtual world position based on a real world head pose.
    // Allows predicting eyes separately based on scanout time.
    Posef				EyePoseFromRealWorldEyePose(const Posef &sensorHeadPose);

    // Handle directional movement. Returns 'true' if movement was processed.
    bool                HandleMoveKey(OVR::KeyCode key, bool down);

    // Movement state; different bits may be set based on the state of keys.
    uint8_t             MoveForward;
    uint8_t             MoveBack;
    uint8_t             MoveLeft;
    uint8_t             MoveRight;
    Vector3f            GamepadMove, GamepadRotate;
	Vector3f			TouchpadMove, TouchpadRotate;
	float				FrameRate = 60;

	Platform::Input::LowPassFilter<VectorMath::Angle<float>, float> BodyYawFilter;

    bool                bMotionRelativeToBody;

	Player();
	~Player();
	void HandleMovement(double dt, Array<Ptr<CollisionModel> >* collisionModels,
		                Array<Ptr<CollisionModel> >* groundCollisionModels, bool shiftDown);

	float DistanceToGround(Array<Ptr<CollisionModel> >* groundCollisionModels);

};

#endif
