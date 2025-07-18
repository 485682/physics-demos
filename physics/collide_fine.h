#pragma once

/**
* this file contains the fine grained collision detection system.
* it is used to return contacts between pairs of primitives.
*
* there are two groups of tests in this file. intersection tests
* use the fastest separating axis method to check if two objects
* intersect, and the collision tests generate the contacts. the
* collision tests typically use the intersection tests as an early
* out.
*/

#include "contacts.h"

// forward declarations of primitive friends
struct intersection_tests;
struct collision_detector;

/**
* represents a primitive to detect collisions against.
*/
struct collision_primitive {

	/**
	* the rigid body that is represented by this primitive.
	*/
	rigid_body * m_body;

	/**
	* the offset of this primitive from the given rigid body.
	*/
	_mat4 m_offset;

	/**
	* calculates the internals for the primitive.
	*/
	void calculateinternals();

	/**
	* this is a convenience function to allow access to the
	* axis vectors in the transform for this primitive.
	*/
	_vec3 getaxis(uint32_t index) const { return m_transform.getaxisvector(index); }

	/**
	* returns the resultant transform of the primitive, calculated from
	* the combined offset of the primitive and the transform
	* (orientation + position) of the rigid body to which it is
	* attached.
	*/
	const _mat4 & gettransform() const { return m_transform; }

	/**
	* the resultant transform of the primitive. this is
	* calculated by combining the offset of the primitive
	* with the transform of the rigid body.
	*/
	_mat4 m_transform;
};

/**
* represents a rigid body that can be treated as a sphere
* for collision detection.
*/
struct collision_sphere : public collision_primitive {
	/**
	* the radius of the sphere.
	*/
	float m_radius;
};

/**
* the plane is not a primitive: it doesn't represent another
* rigid body. it is used for contacts with the immovable
* world geometry.
*/
struct collision_plane {

	/**
	* the plane normal
	*/
	_vec3 m_direction;

	/**
	* the distance of the plane from the origin.
	*/
	float m_offset;
};

/**
* represents a rigid body that can be treated as an aligned bounding
* box for collision detection.
*/
struct collision_box : public collision_primitive {
	/**
	* holds the half-sizes of the box along each of its local axes.
	*/
	_vec3 m_half_size;
};

/**
* a wrapper class that holds fast intersection tests. these
* can be used to drive the coarse collision detection system or
* as an early out in the full collision tests below.
*/
struct intersection_tests {
public:

	static bool sphereandhalfspace( 
		const collision_sphere &sphere, 
		const collision_plane &plane
		);
	static bool sphereandsphere(
		const collision_sphere &one,
		const collision_sphere &two
		);
	static bool boxandbox(
		const collision_box &one,
		const collision_box &two
		);

	/**
	* does an intersection test on an arbitrarily aligned box and a
	* half-space.
	*
	* the box is given as a transform matrix, including
	* position, and a vector of half-sizes for the extend of the
	* box along each local axis.
	*
	* the half-space is given as a direction (i.e. unit) vector and the
	* offset of the limiting plane from the origin, along the given
	* direction.
	*/
	static bool boxandhalfspace( const collision_box &box, const collision_plane &plane);
};


/**
* a helper structure that contains information for the detector to use
* in building its contact data.
*/
struct collision_data {
	/**
	* holds the base of the collision data: the first contact
	* in the array. this is used so that the contact pointer (below)
	* can be incremented each time a contact is detected, while
	* this pointer points to the first contact found.
	*/

	contact * m_contact_array;

	/** holds the contact array to write into. */
	contact * m_contacts;

	/** holds the maximum number of contacts the array can take. */
	int32_t   m_contacts_left;

	/** holds the number of contacts found so far. */
	uint32_t  m_contact_count;

	/** holds the friction value to write into any collisions. */
	float     m_friction;

	/** holds the restitution value to write into any collisions. */
	float     m_restitution;

	/**
	* holds the collision tolerance, even uncolliding objects this
	* close should have collisions generated.
	*/
	float     m_tolerance;

	/**
	* checks if there are more contacts available in the contact
	* data.
	*/
	bool hasmorecontacts() { return m_contacts_left > 0; }

	/**
	* resets the data so that it has no used contacts recorded.
	*/
	void reset( uint32_t maxcontacts) {
		m_contacts_left = maxcontacts;
		m_contact_count = 0;
		m_contacts = m_contact_array;
	}

	/**
	* notifies the data that the given number of contacts have
	* been added.
	*/
	void addcontacts( uint32_t count) {
		// reduce the number of contacts remaining, add number used
		m_contacts_left -= count;
		m_contact_count += count;

		// move the array forward
		m_contacts += count;
	}
};

/**
* a wrapper class that holds the fine grained collision detection
* routines.
*
* each of the functions has the same format: it takes the details
* of two objects, and a pointer to a contact array to fill. it
* returns the number of contacts it wrote into the array.
*/
struct collision_detector {

	static uint32_t sphereandhalfspace(
		const collision_sphere &sphere,
		const collision_plane &plane,
		collision_data *data );

	static uint32_t sphereandtrueplane(
		const collision_sphere &sphere,
		const collision_plane &plane,
		collision_data *data
		);

	static uint32_t sphereandsphere(
		const collision_sphere &one,
		const collision_sphere &two,
		collision_data *data
		);

	/**
	* does a collision test on a collision box and a plane representing
	* a half-space (i.e. the normal of the plane
	* points out of the half-space).
	*/
	static uint32_t boxandhalfspace(
		const collision_box &box,
		const collision_plane &plane,
		collision_data *data
		);

	static uint32_t boxandbox(
		const collision_box &one,
		const collision_box &two,
		collision_data *data
		);

	static uint32_t boxandpoint(
		const collision_box &box,
		const _vec3 &point,
		collision_data *data
		);

	static uint32_t boxandsphere(
		const collision_box &box,
		const collision_sphere &sphere,
		collision_data *data
		);
};

