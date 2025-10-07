#ifndef PFGEN_H
#define	PFGEN_H

#include <vector>

#include "precision.h"
#include "particle.h"

namespace cyclone {

	class ParticleForceGenerator {
	public:
		virtual void updateForce(Particle* particle, real duration) = 0;
	};

	// Holds all the force generators and the particles they apply to
	class ParticleForceRegistry {
	protected:
		// Keeps track of one force generator and the particle it applies to
		struct ParticleForceRegistration {
			Particle* particle;
			ParticleForceGenerator* fg;
		};

		// Golds the list of registrations
		typedef std::vector<ParticleForceRegistration> Registry;
		Registry registrations;
	
	public:
		// Register force generator to apply ti the given particle
		void add(Particle* particle, ParticleForceGenerator* fg);

		// Remove force generator
		void remove(Particle* particle, ParticleForceGenerator* fg);

		// Crears all registrations
		void clear();

		// Calls the force generators to update the forces
		void updateForces(real duration);
	};

	// Force generator that apply gravity to particles
	class ParticleGravity : public ParticleForceGenerator {
		Vector3 gravity;
	public:
		ParticleGravity(const Vector3& gravity);

		virtual void updateForce(Particle* particle, real duration);
	};

	// Particle generator that apply drag
	class ParticleDrag : public ParticleForceGenerator {
		// Velocity drag coefficient
		real k1;

		// Velocity squared drag coefficient
		real k2;

	public:
		// Creates the generator with the given forces
		ParticleDrag(real k1, real k2);

		virtual void updateForce(Particle* particle, real duration);

	};

	// Force generator that applies a spring force
	class ParticleSpring : public ParticleForceGenerator {
		Particle* other; // Particle at the other end of the spring
		real springConstant;
		real restLength;

	public:
		ParticleSpring(Particle* other, real springConstant, real restLength);

		// Applies the spring force to the given particle
		virtual void updateForce(Particle* particle, real duration);
	};
}
#endif// CYCLONE_CORE_H