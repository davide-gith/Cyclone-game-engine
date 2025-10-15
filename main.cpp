#include "cyclone/precision.h"
#include "cyclone/core.h"
#include "cyclone/pfgen.h"
#include "cyclone/particle.h"
#include <iostream>

using namespace cyclone;

int main() {
	Vector3 p;
	Vector3 v(1, 0, 0);

	p += v * 0.016f;

	std::cout << "Position: (" << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;

	// ----- Test spring -----
	Particle a, b;
	ParticleForceRegistry registry;

	ParticleSpring ps(&b, 1.0f, 2.0f);
	registry.add(&a, &ps);

	ParticleSpring psA(&b, 1.0f, 2.0f);
	registry.add(&a, &psA);

	ParticleSpring psB(&a, 1.0f, 2.0f);
	registry.add(&b, &psB);

	return 0;
}