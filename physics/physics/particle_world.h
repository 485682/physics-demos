#pragma once

#include "plinks.h"
#include "pforce_generators.h"

/**
* keeps track of a set of particles, and provides the means to
* update them all.
*/
struct particle_world {

	typedef _array<particle*> _particles;
	typedef _array<particle_contact_generator*> _contact_generators;

	/**
	* holds the particles
	*/
	_particles m_particles;

	/**
	* true if the world should calculate the number of iterations
	* to give the contact resolver at each frame.
	*/
	bool m_calculate_iterations;

	/**
	* holds the force generators for the particles in this world
	*/
	particle_force_registry m_registry;

	/**
	* holds the resolver for contacts
	*/
	particle_contact_resolver m_resolver;

	/**
	* contact generators
	*/
	_contact_generators m_contact_generators;

	/**
	* Holds the list of contacts.
	*/
	particle_contact * m_contacts;

	/**
	* holds the maximum number of contacts allowed (i.e. the
	* size of the contacts array).
	*/
	uint32_t m_max_contacts;
	
	/**
	* creates a new particle simulator that can handle up to the
	* given number of contacts per frame. you can also optionally
	* give a number of contact-resolution iterations to use. if you
	* don't give a number of iterations, then twice the number of
	* contacts will be used.
	*/
	particle_world(uint32_t maxContacts, uint32_t iterations=0);

	/**
	* deletes the simulator.
	*/
	~particle_world();

	/**
	* calls each of the registered contact generators to report
	* their contacts. returns the number of generated contacts.
	*/
	uint32_t generatecontacts();

	/**
	* integrates all the particles in this world forward in time
	* by the given duration.
	*/
	void integrate(float duration);

	/**
	* processes all the physics for the particle world
	*/
	void runphysics(float duration);

	/**
	* initializes the world for a simulation frame. this clears
	* the force accumulators for particles in the world. after
	* calling this, the particles can have their forces for this
	* frame added.
	*/
	void startframe();


	/**
	* returns the list of contact generators
	*/
	_contact_generators& getcontactgenerators();

	/**
	* returns the force registry
	*/
	particle_force_registry& getforceregistry();
};

/**
* a contact generator that takes an vector of particle pointers and
* collides them against the ground.
*/
struct ground_contacts : public particle_contact_generator {
	particle_world::_particles *m_particles;
	void init(particle_world::_particles *particles);

	virtual uint32_t addcontact(particle_contact *contact, uint32_t limit) const;
};