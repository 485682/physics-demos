#pragma once

#include "pcontacts.h"

/**
* links connect two particles together, generating a contact if
* they violate the constraints of their link. it is used as a
* base class for cables and rods, and could be used as a base
* class for springs with a limit to their extension..
*/

struct particle_link : public particle_contact_generator {
	/**
	* holds the pair of particles that are connected by this link.
	*/
	particle* m_contact_particles[2];

	/**
	* returns the current length of the link.
	*/
	float currentlength() const;
};

/**
* cables link a pair of particles, generating a contact if they
* stray too far apart.
*/
struct particle_cable : public particle_link {

	/**
	* holds the maximum length of the cable.
	*/
	float m_max_length;

	/**
	* holds the restitution (bounciness) of the cable.
	*/
	float m_restitution;

	/**
	* fills the given contact structure with the contact needed
	* to keep the cable from over-extending.
	*/
	virtual uint32_t addcontact(particle_contact *contact, uint32_t limit) const;
};

/**
* rods link a pair of particles, generating a contact if they
* stray too far apart or too close.
*/
struct particle_rod : public particle_link {

	/**
	* holds the length of the rod.
	*/
	float m_length;
	/**
	* fills the given contact structure with the contact needed
	* to keep the rod from extending or compressing.
	*/
	virtual uint32_t addcontact(particle_contact *contact, uint32_t limit) const;
};

/**
* constraints are just like links, except they connect a particle to
* an immovable anchor point.
*/
struct particle_constraint : public particle_contact_generator {

	/**
	* holds the particles connected by this constraint.
	*/
	particle* m_particle;

	/**
	* the point to which the particle is anchored.
	*/
	_vec3 m_anchor;

	/**
	* returns the current length of the link.
	*/
	float currentlength() const;
};

/**
* cables link a particle to an anchor point, generating a contact if they
* stray too far apart.
*/
struct particle_cable_constraint : public particle_constraint {

	/**
	* holds the maximum length of the cable.
	*/
	float m_max_length;

	/**
	* holds the restitution (bounciness) of the cable.
	*/
	float m_restitution;

public:
	/**
	* fills the given contact structure with the contact needed
	* to keep the cable from over-extending.
	*/
	virtual uint32_t addcontact(particle_contact *contact, uint32_t limit) const;
};

/**
* rods link a particle to an anchor point, generating a contact if they
* stray too far apart or too close.
*/
struct particle_rod_constraint : public particle_constraint {

	/**
	* holds the length of the rod.
	*/
	float m_length;

	/**
	* fills the given contact structure with the contact needed
	* to keep the rod from extending or compressing.
	*/
	virtual uint32_t addcontact(particle_contact *contact, uint32_t limit) const;
};