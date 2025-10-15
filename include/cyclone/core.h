#ifndef CYCLONE_CORE_H
#define	CYCLONE_CORE_H

#include "precision.h"

namespace cyclone {

	class Vector3 {
	public:
		real x;
		real y;
		real z;
	private:
		real pad; // Padding to ensure alignment

	public:
		// The default constructor creates a zero vector
		Vector3() : x(0), y(0), z(0), pad(0) {}

		// Parametric constructor
		Vector3(const real x, const real y, const real z) : x(x), y(y), z(z) {}

		// Function that flips all the components of the vector
		void invert() {
			x = -x;
			y = -y;
			z = -z;
		}

		// Function thet gets the magnitude/length of the vector
		real magnitude() const {
			return real_sqrt(x * x + y * y + z * z);
		}

		// Function that gets the squared magnitude/length of the vector
		real squareMagnitude() const {
			return (x * x + y * y + z * z);
		}

		// Turn a non-zero vector into a unit vector
		void normalize() {
			real l = magnitude();

			if (l > 0) {
				(*this) *= ((real)1) / l;
			}
		}

		// Multiplies this vector by the given scalar
		void operator*=(const real value) {
			x *= value;
			y *= value;
			z *= value;
		}

		// Returns a copy of this vector scaled to the given value
		Vector3 operator*(const real value) const {
			return Vector3(x * value, y * value, z * value);
		}

		// Adds the given vector to this
		void operator+=(const Vector3& v) {
			x += v.x;
			y += v.y;
			z += v.z;
		}

		// Return the value of the given vector added to this
		Vector3 operator+(const Vector3& v) const {
			return Vector3(x + v.x, y + v.y, z + v.z);
		}

		// Subtracts the given vector from this
		void operator -=(const Vector3& v) {
			x -= v.x;
			y -= v.y;
			z -= v.z;
		}

		// Return the value of the given vector subtracted from this
		Vector3 operator -(const Vector3& v) const {
			return Vector3(x - v.x, y - v.y, z - v.z);
		}

		// Add scaled vector
		void AddScaledVector(const Vector3& vector, real scale) {
			x += vector.x * scale;
			y += vector.y * scale;
			z += vector.z * scale;
		}

		// Return the result of the componet-wise product of this vector with the given vector
		Vector3 componentProduct(const Vector3& vector) const {
			return Vector3(x * vector.x, y * vector.y, z * vector.z);
		}

		// Compute the result of the componet-wise product in-place
		void compontProductUpdate(const Vector3& vector) {
			x *= vector.x;
			y *= vector.y;
			z *= vector.z;
		}

		// Return the scalar product of this vector with the given vector
		real ScalarProduct(const Vector3& vector) const {
			return (x * vector.x + y * vector.y + z * vector.z);
		}

		// Scalar product overload
		real operator*(const Vector3& vector) const {
			return (x * vector.x + y * vector.y + z * vector.z);
		}

		// Return the vector product (cross product) of this vector with the given vector
		Vector3 VectorProduct(const Vector3& vector) const {
			return Vector3(
				y * vector.z - z * vector.y,
				z * vector.x - x * vector.z,
				x * vector.y - y * vector.x);
		}

		// Vector product overload in-place
		void operator%=(const Vector3& vector) {
			*this = VectorProduct(vector);
		}

		// Vector product overload
		Vector3 operator%(const Vector3& vector) const {
			return Vector3(
				y * vector.z - z * vector.y,
				z * vector.x - x * vector.z,
				x * vector.y - y * vector.x);
		}

		// Zero the components values
		void clear() {
			x = 0;
			y = 0;
			z = 0;
		}
	};


	// A 3x3 matrix
	class Matrix3 {
	public:
		real data[9];

		Matrix3()
		{
			data[0] = data[1] = data[2] = 0;
			data[3] = data[4] = data[5] = 0;
			data[6] = data[7] = data[8] = 0;
		}

		Matrix3(real c0, real c1, real c2, real c3, real c4, real c5,
			real c6, real c7, real c8)
		{
			data[0] = c0; data[1] = c1; data[2] = c2;
			data[3] = c3; data[4] = c4; data[5] = c5;
			data[6] = c6; data[7] = c7; data[8] = c8;
		}

		Matrix3 operator*(const Matrix3& o) const {
			return Matrix3(
				data[0] * o.data[0] + data[1] * o.data[3] + data[2] * o.data[6],
				data[0] * o.data[1] + data[1] * o.data[4] + data[2] * o.data[7],
				data[0] * o.data[2] + data[1] * o.data[5] + data[2] * o.data[8],

				data[3] * o.data[0] + data[4] * o.data[3] + data[5] * o.data[6],
				data[3] * o.data[1] + data[4] * o.data[4] + data[5] * o.data[7],
				data[3] * o.data[2] + data[4] * o.data[5] + data[5] * o.data[8],

				data[6] * o.data[0] + data[7] * o.data[3] + data[8] * o.data[6],
				data[6] * o.data[1] + data[7] * o.data[4] + data[8] * o.data[7],
				data[6] * o.data[2] + data[7] * o.data[5] + data[8] * o.data[8]
			);
		}

		void setInverse(const Matrix3& m) {
			real t4 = m.data[0] * m.data[4];
			real t6 = m.data[0] * m.data[5];
			real t8 = m.data[1] * m.data[3];
			real t10 = m.data[2] * m.data[3];
			real t12 = m.data[1] * m.data[6];
			real t14 = m.data[2] * m.data[6];

			// Determinant
			real t16 = (t4 * m.data[8] - t6 * m.data[7] - t8 * m.data[8] + t10 * m.data[7] + t12 * m.data[5] - t14 * m.data[4]);

			// If the determinant is 0 the invers doesn't exists
			if (t16 == (real)0.0f) {
				return;
			}
			real t17 = 1 / t16;

			data[0] = (m.data[4] * m.data[8] - m.data[5] * m.data[7]) * t17;
			data[1] = -(m.data[1] * m.data[8] - m.data[2] * m.data[7]) * t17;
			data[2] = (m.data[1] * m.data[5] - m.data[2] * m.data[4]) * t17;
			data[3] = -(m.data[3] * m.data[8] - m.data[5] * m.data[6]) * t17;
			data[4] = (m.data[0] * m.data[8] - t14) * t17;
			data[5] = -(t6 - t10) * t17;
			data[6] = (m.data[3] * m.data[7] - m.data[4] * m.data[6]) * t17;
			data[7] = -(m.data[0] * m.data[7] - t12) * t17;
			data[8] = (t4 - t8) * t17;
		}

		Matrix3 inverse() const {
			Matrix3 result;
			result.setInverse(*this);
			return result;
		}

		void invert() {
			setInverse(*this);
		}

		void setTranspose(const Matrix3& m) {
			data[0] = m.data[0];
			data[1] = m.data[3];
			data[2] = m.data[6];

			data[3] = m.data[1];
			data[4] = m.data[4];
			data[5] = m.data[7];

			data[6] = m.data[2];
			data[7] = m.data[5];
			data[8] = m.data[8];
		}

		Matrix3 transpose() {
			Matrix3 result;
			result.setTranspose(*this);
			return result;
		}
	};

	// a 3x4 matrix
	class Matrix4 {
	public:
		real data[12];

		// Identity matrix
		Matrix4()
		{
			data[1] = data[2] = data[3] = data[4] = data[6] = data[7] = data[8] = data[9] = data[11] = 0;
			data[0] = data[5] = data[10] = 1;
		}

		Vector3 operator*(const Vector3& vector) const {
			return Vector3(
				vector.x * data[0] + vector.y * data[1] + vector.z * data[2] + data[3],
				vector.x * data[4] + vector.y * data[5] + vector.z * data[6] + data[7],
				vector.x * data[8] + vector.y * data[9] + vector.z * data[10] + data[11]
			);
		}

		Matrix4 operator*(const Matrix4& o) const {
			Matrix4 result;

			result.data[0] = (o.data[0] * data[0]) + (o.data[4] * data[1]) + (o.data[8] * data[2]);
			result.data[4] = (o.data[0] * data[4]) + (o.data[4] * data[5]) + (o.data[8] * data[6]);
			result.data[8] = (o.data[0] * data[8]) + (o.data[4] * data[9]) + (o.data[8] * data[10]);

			result.data[1] = (o.data[1] * data[0]) + (o.data[5] * data[1]) + (o.data[9] * data[2]);
			result.data[5] = (o.data[1] * data[4]) + (o.data[5] * data[5]) + (o.data[9] * data[6]);
			result.data[9] = (o.data[1] * data[8]) + (o.data[5] * data[9]) + (o.data[9] * data[10]);

			result.data[2] = (o.data[2] * data[0]) + (o.data[6] * data[1]) + (o.data[10] * data[2]);
			result.data[6] = (o.data[2] * data[4]) + (o.data[6] * data[5]) + (o.data[10] * data[6]);
			result.data[10] = (o.data[2] * data[8]) + (o.data[6] * data[9]) + (o.data[10] * data[10]);

			result.data[3] = (o.data[3] * data[0]) + (o.data[7] * data[1]) + (o.data[11] * data[2]) + data[3];
			result.data[7] = (o.data[3] * data[4]) + (o.data[7] * data[5]) + (o.data[11] * data[6]) + data[7];
			result.data[11] = (o.data[3] * data[8]) + (o.data[7] * data[9]) + (o.data[11] * data[10]) + data[11];

			return result;
		}

		real getDeterminant() const;
		
		void setInverse(const Matrix4& m);

		Matrix4 inverse() const {
			Matrix4 result;
			result.setInverse(*this);
			return result;
		}

		void invert() {
			setInverse(*this);
		}
	};

}
#endif// CYCLONE_CORE_H