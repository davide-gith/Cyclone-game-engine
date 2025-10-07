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
			return (x* vector.x + y * vector.y + z * vector.z);
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
}
#endif// CYCLONE_CORE_H