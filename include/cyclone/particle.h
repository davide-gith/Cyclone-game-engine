#ifndef PARTICLE_H
#define PARTICLE_H

#include "core.h"
#include "precision.h"

namespace cyclone {
	class Particle {
	protected:
		Vector3 position;
		Vector3 velocity;
		Vector3 acceleration;
		real inverseMass;
		real damping; // Holds the amount of damping applied to linear motion
		Vector3 forceAccum; // Holds the accumulated force to be applied at the next simulation iteration
	public:
		void setMass(const real mass);
		real getMass() const;
		
		void setInverseMass(const real inverseMass);
		real getInverseMass() const;

		// Integrate the particle forward in time (Newton-Euler integration)
		void integrate(real duration);

		// Cleat the forces applied to the particles
		void clearAccumulator();

		// Add force to the particle
		void addForce(const Vector3& force);

		// Return true if the mass is not infinite
		bool hasFiniteMass() const;

		// Get position
		void getPosition(Vector3* position) const;

		Vector3 getPosition() const;

		// Get velocity of the particle in-place
		void getVelocity(Vector3* velocity) const;

		// Get velocity of the particle
		Vector3 getVelocity() const;
	};
}

#endif//PARTICLE_H