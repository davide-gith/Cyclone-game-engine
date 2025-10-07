#include <assert.h>
#include <cyclone/particle.h>
#include "cyclone/precision.h"

using namespace cyclone;

void Particle::setMass(const real mass) {
	assert(mass != 0);
	Particle::inverseMass = ((real)1.0) / mass;
}

real Particle::getMass() const {
	if (inverseMass == 0) {
		return REAL_MAX;
	}
	else {
		return ((real)1.0)/inverseMass;
	}
}

void Particle::setInverseMass(const real inverseMass) {
	Particle::inverseMass = inverseMass;
}

real Particle::getInverseMass() const {
	return inverseMass;
}

void Particle::integrate(real duration) {
	assert(duration > 0.0);

	// Update linear position
	position.AddScaledVector(velocity, duration);

	// Acceleration from the forse
	Vector3 resultingAcceleration = acceleration;
	resultingAcceleration.AddScaledVector(forceAccum, inverseMass);

	// Update velocity from linear acceleration
	velocity.AddScaledVector(resultingAcceleration, duration);

	// Drag
	velocity *= real_pow(damping, duration);

	// Clear the forces
	clearAccumulator();
}

void Particle::clearAccumulator() {
	forceAccum.clear();
}

void Particle::addForce(const Vector3& force) {
	forceAccum += force;
}

bool Particle::hasFiniteMass() const {
	return inverseMass >= 0.0f;
}

void Particle::getPosition(Vector3* position) const {
	*position = Particle::position;
}

Vector3 Particle::getPosition() const {
	return position;
}

void Particle::getVelocity(Vector3* velocity) const{
	*velocity = Particle::velocity;
}

Vector3 Particle::getVelocity() const {
	return velocity;
}