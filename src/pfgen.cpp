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

ParticleSpring::ParticleSpring(Particle* other, real sc, real r1) : other(other), springConstant(sc), restLength(r1) {
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