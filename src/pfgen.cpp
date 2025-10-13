#include "cyclone/pfgen.h"

using namespace cyclone;

void ParticleForceRegistry::updateForces(real duration) {
	Registry::iterator i = registrations.begin();
	for (; i != registrations.end(); ++i) {
		i->fg->updateForce(i->particle, duration);
	}
}

void ParticleForceRegistry::add(Particle* particle, ParticleForceGenerator *fg) {
	ParticleForceRegistry::ParticleForceRegistration registration;
	registration.particle = particle;
	registration.fg = fg;
	registrations.push_back(registration);
}

ParticleGravity::ParticleGravity(const Vector3& gravity) : gravity(gravity) {
}

void ParticleGravity::updateForce(Particle* particle, real duration) {
	// Check: infinite mass?
	if (!particle->hasFiniteMass()) {
		return;
	}

	// Apply the mass-scaled force to the particle
	particle->addForce(gravity * particle->getMass());
}

ParticleDrag::ParticleDrag(real k1, real k2) : k1(k1), k2(k2) {
}

void ParticleDrag::updateForce(Particle* particle, real duration) {
	Vector3 force;
	particle->getVelocity(&force);

	// Compute drag coefficient
	real dragCoeff = force.magnitude();
	dragCoeff = k1 * dragCoeff + k2 * dragCoeff * dragCoeff;

	// Compute final force and apply
	force.normalize();
	force *= -dragCoeff;
	particle->addForce(force);
}

ParticleSpring::ParticleSpring(Particle* other, real sc, real r1) : other(other), springConstant(sc), restLength(r1) 
{
}

void ParticleSpring::updateForce(Particle* particle, real duration) {
	Vector3 force;

	particle->getPosition(&force);
	force -= other->getPosition();

	// Compute magnitude of the force
	real magnitude = force.magnitude();
	magnitude = real_abs(magnitude - restLength);
	magnitude *= springConstant;

	// Compute final force and apply it
	force.normalize();
	force *= -magnitude;
	particle->addForce(force);
}


ParticleAnchoredSpring::ParticleAnchoredSpring(Vector3* anchor, real sc, real rl) : anchor(anchor), springConstant(sc), restLength(rl)
{
}

void ParticleAnchoredSpring::updateForce(Particle* particle, real duration) {
	// Calculate the vector of the spring
	Vector3 force;
	particle->getPosition(&force);
	force -= *anchor;

	// Magnitude of the force
	real magnitude = force.magnitude();
	magnitude = real_abs(magnitude -restLength);
	magnitude *= springConstant;

	// Final force to apply
	force.normalize();
	force *= -magnitude;
	particle->addForce(force);
}

ParticleBungee::ParticleBungee(Particle* other, real sc, real rl) : other(other), springConstant(sc), restLength(rl)
{
}

void ParticleBungee::updateForce(Particle* particle, real duration) {
	// Calculate the vector of the spring
	Vector3 force;
	particle->getPosition(&force);
	force -= other->getPosition();

	// Check if there' a compression
	real magnitude = force.magnitude();
	if (magnitude <= restLength) {
		return;
	}

	// Magnitude of the force
	magnitude = springConstant * (magnitude - restLength);

	// Final force to apply
	force.normalize();
	force *= -magnitude;
	particle->addForce(force);
}

ParticleBuoyancy::ParticleBuoyancy(real maxDepth, real volume, real waterHeight, real liquidDensity) : maxDepth(maxDepth), volume(volume), waterHeight(waterHeight), liquidDensity(liquidDensity)
{
}

void ParticleBuoyancy::updateForce(Particle* particle, real duration) {
	// Calculate the submersion depth
	real depth = particle->getPosition().y;

	// Check if out of the water
	if (depth >= waterHeight + maxDepth) {
		return;
	}

	Vector3 force(0, 0, 0);

	// Check maximum depth
	if (depth <= waterHeight - maxDepth) {
		force.y = liquidDensity * volume;
		particle->addForce(force);
		return;
	}

	// Partial submersion
	force.y = liquidDensity * volume * (depth - maxDepth - waterHeight) / (2 * maxDepth);
	particle->addForce(force);
}