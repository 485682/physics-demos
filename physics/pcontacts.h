#pragma once

#include "particle.h"

/*
* forward declaration, see full declaration below for complete
* documentation.
*/
struct Particle_contact_resolver;

/**
* a contact represents two objects in contact (in this case
* particlecontact representing two particles). resolving a
* contact removes their interpenetration, and applies sufficient
* impulse to keep them apart. colliding bodies may also rebound.
*/
struct particle_contact {

	/**
	* holds the particles that are involved in the contact. the
	* second of these can be null, for contacts with the scenery.
	*/
	particle* m_contact_particles[2];

	/**
	* holds the normal restitution coefficient at the contact.
	*/
	float m_restitution;

	/**
	* holds the direction of the contact in world coordinates.
	*/
	_vec3 m_contact_normal;

	/**
	* holds the depth of penetration at the contact.
	*/
	float m_penetration;

	/**
	* holds the amount each particle is moved by during interpenetration
	* resolution.
	*/
	_vec3 m_particle_movement[2];

	/**
	* resolves this contact, for both velocity and interpenetration
	*/
	void resolve(float duration);

	/**
	* calculates the separating velocity at this contact
	*/
	float calculateseparatingvelocity() const;

	/**
	* handles the impulse calculations for this collision
	*/
	void resolvevelocity(float duration);

	/**
	* handles the interpenetration resolution for this contact
	*/
	void resolveinterpenetration(float duration);

};

/**
* the contact resolution routine for particle contacts. one
* resolver instance can be shared for the whole simulation.
*/
struct particle_contact_resolver {

	/**
	* holds the number of iterations allowed
	*/
	uint32_t m_iterations;

	/**
	* this is a performance tracking value - we keep a record
	* of the actual number of iterations used.
	*/
	uint32_t m_iterations_used;

	/**
	* creates a new contact resolver
	*/
	particle_contact_resolver(uint32_t iterations);

	/**
	* sets the number of iterations that can be used
	*/
	void setiterations(uint32_t iterations);

	/**
	* resolves a set of particle contacts for both penetration
	**/
	void resolvecontacts(particle_contact *contactarray,uint32_t numcontacts,float duration);
};

/**
* this is the basic polymorphic interface for contact generators
* applying to particles.
*/
struct particle_contact_generator {

	/**
	* fills the given contact structure with the generated
	* contact. the contact pointer should point to the first
	* available contact in a contact array, where limit is the
	* maximum number of contacts in the array that can be written
	* to. the method returns the number of contacts that have
	* been written.
	*/
	virtual uint32_t addcontact(particle_contact *contact, uint32_t limit) const = 0;
};