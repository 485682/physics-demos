#pragma once

#include "contacts.h"

/**
* joints link together two rigid bodies and make sure they do not
* separate.  in a general phyiscs engine there may be many
* different types of joint, that reduce the number of relative
* degrees of freedom between two objects. this joint is a common
* position joint: each object has a location (given in
* body-coordinates) that will be kept at the same point in the
* simulation.
*/
struct joint : public contact_generator {
public:
	/**
	* holds the two rigid bodies that are connected by this joint.
	*/
	rigid_body* m_body[2];

	/**
	* holds the relative location of the connection for each
	* body, given in local coordinates.
	*/
	_vec3 m_position[2];

	/**
	* holds the maximum displacement at the joint before the
	* joint is considered to be violated. this is normally a
	* small, epsilon value.  it can be larger, however, in which
	* case the joint will behave as if an inelastic cable joined
	* the bodies at their joint locations.
	*/
	float m_error;

	/**
	* configures the joint in one go
	*/
	void set(
		rigid_body *a, const _vec3& a_pos,
		rigid_body *b, const _vec3& b_pos,
		float error
		);

	/**
	* generates the contacts required to restore the joint if it
	* has been violated.
	*/
	uint32_t addcontact(contact *contact, uint32_t limit) const;
};