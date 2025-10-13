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
		real k1; // Velocity drag coefficient
		real k2; // Velocity squared drag coefficient

	public:
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
		virtual void updateForce(Particle* particle, real duration);
	};

	// Force generator that applied a spring forcem where one end is attached to a fixed point in space
	class ParticleAnchoredSpring : public ParticleForceGenerator {
		Vector3* anchor; 		// Location of the anchored end of the spring
		real springConstant;
		real restLength;

	public:
		ParticleAnchoredSpring(Vector3* anchor, real springConstant, real restLength);
		virtual void updateForce(Particle* particle, real duration);
	};

	// Force generator that applies a bungee force
	class ParticleBungee : ParticleForceGenerator {
		Particle* other; // Particle at the other end of the spring
		real springConstant;
		real restLength;

	public:
		ParticleBungee(Particle* other, real springConstant, real restLength);
		virtual void updateForce(Particle* particle, real duration);
	};

	// Force generator that applies a buoyancy force
	class ParticleBuoyancy : public ParticleForceGenerator {
		real maxDepth; // max depth of the object before it is fully submerged
		real volume;
		real waterHeight;
		real liquidDensity; // Density of the liquid (water = 1000kg/m^3)

	public:
		ParticleBuoyancy(real maxDepth, real volume, real waterHeight, real liquidDensity = 1000.0f);
		virtual void updateForce(Particle* particle, real duration);
	};
}

#endif// CYCLONE_CORE_H