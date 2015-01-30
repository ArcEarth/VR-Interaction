#ifndef MATHES_H
#define MATHES_H
#pragma once

#include "MathHelper.h"

/*
#include <DirectXMath.h>
using namespace DirectX;

class Vector2;
class Vector3;
class Quaternion;

class Vector3
{
private:
	//The 4D-register for efficient caculating
	XMVECTOR vtr;
public:
	Vector3(void){
		vtr = XMVectorSet( 0.0f, 0.0f, 0.0f, 1.0f );
	}
	Vector3(const XMVECTOR &vector){
		vtr = vector;
	}
	Vector3(float x,float y,float z){
		vtr = XMVectorSet( x, y, z, 1.0f );
	}
	Vector3(const Vector4 &vec){
		vtr = XMVectorSet( vec.x, vec.y, vec.z, 1.0f );
	}
	Vector3(const Vector2 &vector2);
	~Vector3(void);

	inline void Set(float x,float y,float z){
		vtr = XMVectorSet( x, y, z, 1.0f );
	}

	inline Vector3& operator += (const Vector3& rhs){
		vtr += rhs;
		return *this;
	}
	inline Vector3& operator -= (const Vector3& rhs){
		vtr -= rhs;
		return *this;
	}
	inline Vector3& operator *= (const float rhs){
		vtr *= rhs;
		return *this;
	}
	// Each Product of two 3d-vector
	inline Vector3& operator *= (const Vector3& rhs){
		vtr = vtr * rhs.vtr;
		return *this;
	}
	// Equals to transform this with rhs
	inline Vector3& operator *= (const XMMATRIX& rhs){
		vtr = XMVector3TransformCoord(vtr,rhs);
		return *this;
	}
	// Cross product of two 3d-vector
	inline Vector3& operator ^= (const Vector3& rhs){
		vtr = XMVector3Cross(vtr,rhs.vtr);
		return *this;
	}
	inline Vector3& operator /= (const float rhs){
		vtr /= rhs;
		return *this;
	}

	inline Vector3 operator + (const Vector3 & rhs) const{
		return vtr + rhs.vtr;
	}
	inline Vector3 operator - (const Vector3 & rhs) const{
		return vtr - rhs.vtr;
	}
	inline Vector3 operator * (const float rhs) const{
		return vtr*rhs;
	}
	// Dot product of two 3d-vector
	inline float operator * (const Vector3& rhs) const{
		return XMVectorGetX(XMVector3Dot(vtr,rhs.vtr));
	}
	// Transform this vector with a matrix
	inline Vector3 operator * (const XMMATRIX& rhs) const{
		return XMVector3TransformCoord(vtr,rhs);
	}
	// Cross product of two 3d-vector
	inline Vector3 operator ^ (const Vector3& rhs) const{
		return XMVector3Cross(vtr,rhs.vtr);
	}
	inline Vector3 operator / (const float rhs) const{
		return vtr / rhs;
	}

	//unary operator positive
	inline Vector3 operator + () const{
		return vtr;
	}
	//unary operator negetive
	inline Vector3 operator - () const{
		return vtr*(-1);
	}

	inline bool operator == (const Vector3& rhs) const{
		return XMVector3Equal(vtr,rhs.vtr);
	}
	inline bool operator != (const Vector3& rhs) const{
		return !(*this == rhs);
	}

	inline float Length() const{
		return XMVectorGetX(XMVector3Length(vtr));
	}
	inline float LengthSq() const{
		return XMVectorGetX(XMVector3LengthSq(vtr));
	}

	// This method will normailize itselft and return a reference to itself.
	inline Vector3& Normailize(){
		vtr = XMVector3Normalize(vtr);
		return *this;
	}

	inline Vector3 Rotate(const XMVECTOR& RotateQuaternion) const{
		return XMVector3Rotate(vtr,RotateQuaternion);
	}

	inline operator const XMFLOAT3 () const{
		XMFLOAT3 f3;
		XMStoreFloat3(&f3,vtr);
		return f3;
	}

	inline operator XMFLOAT3 (){
		XMFLOAT3 f3;
		XMStoreFloat3(&f3,vtr);
		return f3;
	}

	inline operator const XMFLOAT4 () const{
		XMFLOAT4 f4;
		XMStoreFloat4(&f4,vtr);
		return f4;
	}

	inline operator XMFLOAT4 (){
		XMFLOAT4 f4;
		XMStoreFloat4(&f4,vtr);
		return f4;
	}

	inline operator XMVECTOR& (){
		return vtr;
	}

	inline operator const XMVECTOR& () const{
		return vtr;
	}

	friend Vector3 operator * (const float lhs,const Vector3&rhs);
};

Vector3 operator * (const float lhs,const Vector3&rhs){
	return lhs * rhs.vtr;
}

class Quaternion
{
private:
	XMVECTOR vtr;
public:
	Quaternion()
	{
		vtr = Identity();
	}
	Quaternion(XMVECTOR &vector){
		vtr = vector;
	}
	Quaternion(const float* array){
		vtr = XMLoadFloat4(&XMFLOAT4(array));
	}

	//// Construct a Quaternion from a 3D-Vector for compute
	//explicit Quaternion(const XMFLOAT3 &vector)
	//{
	//	vtr = XMVectorSet(0.0f,vector.x,vector.y,vector.z);
	//}

	// Construct a Quaternion from Rotate Axias and Angle
	Quaternion(const Vector3 &Axias , const float Angle)
	{
		vtr = XMQuaternionRotationAxis((XMVECTOR)Axias,Angle);
	}
	// Construct a Quaternion from Roll,Pitch,Yaw
	Quaternion(const float Roll , const float Pitch , const float Yaw)
	{
		vtr = XMQuaternionRotationRollPitchYaw(Roll,Pitch,Yaw);
	}
	~Quaternion()
	{
	}

public:
	inline static const Quaternion Identity(){
		return XMQuaternionIdentity();
	}

public:
	inline void SetIdentity(){
		vtr = XMQuaternionIdentity();
	}

	inline Quaternion Conjugate() const{
		return XMQuaternionConjugate(vtr);
	}

	inline Quaternion Inverse() const{
		return XMQuaternionInverse(vtr);
	}

	inline Quaternion& operator *= (const Quaternion& rhs)
	{
		vtr = XMQuaternionMultiply(vtr,rhs.vtr);
	}

	inline Quaternion operator * (const Quaternion& rhs) const
	{
		return XMQuaternionMultiply(vtr,rhs.vtr);
	}

	inline operator XMFLOAT4 ()
	{
		XMFLOAT4 f4;
		XMStoreFloat4(&f4,vtr);
		return f4;
	}

	inline operator const XMFLOAT4 () const
	{
		XMFLOAT4 f4;
		XMStoreFloat4(&f4,vtr);
		return f4;
	}

	inline operator XMVECTOR& ()
	{
		return vtr;
	}

	inline operator const XMVECTOR& () const
	{
		return vtr;
	}
};


// Helper class for descript the position and oritation of an object
// The object have 6-Dimention freedom
class RigidObject
{
public:
	RigidObject() : m_Position(),m_Orientation()
	{
	}

	inline const XMMATRIX WorldMatrix() const
	{
		return XMMatrixRotationQuaternion(m_Orientation)*XMMatrixTranslationFromVector(m_Position);
	}

	inline const XMMATRIX InverseWorldMatrix() const
	{
		return XMMatrixTranslationFromVector(-m_Position)*XMMatrixRotationQuaternion(m_Orientation.Inverse());
	}

	const Vector3 &Position() const
	{
		return m_Position;
	}
	const Quaternion &Orientation() const{
		return m_Orientation;
	}

	void Identitify(){
		m_Position.Set(0.0f,0.0f,0.0f);
		m_Orientation.SetIdentity();
	}
	void SetPosition(const Vector3 &Position){
		m_Position = Position;
	}
	void SetOrientation(const Quaternion &Orientation){
		m_Orientation = Orientation;
	}

	void Translate(const Vector3 &Offset){
		m_Position += Offset;
	}
	void Rotate(const Quaternion &RotateQuaterion){
		m_Orientation *= RotateQuaterion;
	}
	void Rotate(const Vector3 &Axias , const float Angle){
		m_Orientation *= Quaternion(Axias,Angle);
	}
	void Rotate(const float Roll , const float Pitch , const float Yaw){
		m_Orientation *= Quaternion(Roll,Pitch,Yaw);
	}

public:
	const static RigidObject Identity;
private:
	Vector3 m_Position;
	Quaternion m_Orientation;

#if Enable_Physics
	// The Translational Velocity and Acceleration
	Vector3 m_Velocity,m_Acceleration;
	// The Angular Velocity and Acceleration
	Vector3 m_AngularVelocity,m_AngularAcceleration;
#endif // Enable_Physics
};  */
#endif // !MATHES_H
