#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

// Unity Vector3
struct Vector3
{
	float x;
	float y;
	float z;

	Vector3() = default;

	Vector3(float x, float y, float z) : x(x), y(y), z(z)
	{}

	explicit Vector3(Eigen::Vector3f value) : x(value(0)), y(value(1)), z(value(2))
	{}

	// Do not overload cast operator, this causes conflicts with Eigen's cast operator on some versions
	// explicit operator Eigen::RowVector3f() const;
	// explicit operator Eigen::Vector3f() const;
	Eigen::Vector3f AsEigen() const;

	Eigen::RowVector3f AsEigenRow() const;

	inline static Vector3 Zero()
	{ return Vector3(0.f, 0.f, 0.f); }
};

struct Quaternion
{
	float x;
	float y;
	float z;
	float w;

	Quaternion() = default;

	Quaternion(float x, float y, float z, float w) : x(x), y(y), z(z), w(w)
	{}

	explicit Quaternion(Eigen::Quaternionf& q) : x(q.x()), y(q.y()), z(q.z()), w(q.w())
	{}

	/**
	 * @warning Eigen has a different ordering of the values, handled safely by this function.
	 * We cannot simply reinterpret the bits.
	 */
	inline Eigen::Quaternionf AsEigen() const
	{ return Eigen::Quaternionf(w, x, y, z); }

	inline static Quaternion Identity()
	{ return Quaternion(0.f, 0.f, 0.f, 1.0f); }
};

struct MeshDataNative
{
    float* VPtr;
    float* NPtr;
    int* FPtr;

    // Vector3 com;
	// float Mass;

	// int materialType;
    // float mu;
    // float lambda;
    
    int VSize;
    int FSize;
};