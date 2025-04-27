#pragma once

/**
* this file contains the contact resolution system for cyclone,
* although it is called the contact resolution system, it handles
* collisions, contacts (sliding and resting), and constraints (such
* as joints).
*
* the resolver uses an iterative satisfaction algorithm; it loops
* through each contact and tries to resolve it. this is a very fast
* algorithm but can be unstable when the contacts are highly
* inter-related.
*/
#include "body.h"
/*
* forward declaration, see full declaration below for complete
* documentation.
*/
struct contact_resolver;

/**
* a contact represents two bodies in contact. resolving a
* contact removes their interpenetration, and applies sufficient
* impulse to keep them apart. colliding bodies may also rebound.
* contacts can be used to represent positional joints, by making
* the contact constraint keep the bodies in their correct
* orientation.
*
* it can be a good idea to create a contact object even when the
* contact isn't violated. because resolving one contact can violate
* another, contacts that are close to being violated should be
* sent to the resolver; that way if one resolution moves the body,
* the contact may be violated, and can be resolved. if the contact
* is not violated, it will not be resolved, so you only loose a
* small amount of execution time.
*
* the contact has no callable functions, it just holds the contact
* details. to resolve a set of contacts, use the contact resolver
* class.
*/

struct contact {
	/**
	* holds the bodies that are involved in the contact. the
	* second of these can be null, for contacts with the scenery.
	*/
	rigid_body * m_body[2];

	/**
	* holds the lateral friction coefficient at the contact.
	*/
	float m_friction;

	/**
	* holds the normal restitution coefficient at the contact.
	*/
	float m_restitution;

	/**
	* holds the position of the contact in world coordinates.
	*/
	_vec3 m_contact_point;

	/**
	* holds the direction of the contact in world coordinates.
	*/
	_vec3 m_contact_normal;

	/**
	* holds the depth of penetration at the contact point. if both
	* bodies are specified then the contact point should be midway
	* between the inter-penetrating points.
	*/
	float m_penetration;

	/**
	* sets the data that doesn't normally depend on the position
	* of the contact (i.e. the bodies, and their material properties).
	*/
	void setbodydata(rigid_body* one, rigid_body *two,float friction, float restitution);

	/**
	* a transform matrix that converts co-ordinates in the contact's
	* frame of reference to world co-ordinates. the columns of this
	* matrix form an orthonormal set of vectors.
	*/
	_mat3 m_contact_to_world;

	/**
	* holds the closing velocity at the point of contact. this is set
	* when the calculateinternals function is run.
	*/
	_vec3 m_contact_velocity;

	/**
	* holds the required change in velocity for this contact to be
	* resolved.
	*/
	float m_desired_delta_velocity;

	/**
	* holds the world space position of the contact point relative to
	* centre of each body. this is set when the calculateinternals
	* function is run.
	*/
	_vec3 m_relative_contact_position[2];

	/**
	* calculates internal data from state data. this is called before
	* the resolution algorithm tries to do any resolution. it should
	* never need to be called manually.
	*/
	void calculateinternals( float duration);

	/**
	* reverses the contact. this involves swapping the two rigid bodies
	* and reversing the contact normal. the internal values should then
	* be recalculated using calculateinternals (this is not done
	* automatically).
	*/
	void swapbodies();

	/**
	* updates the awake state of rigid bodies that are taking
	* place in the given contact. a body will be made awake if it
	* is in contact with a body that is awake.
	*/
	void matchawakestate();

	/**
	* calculates and sets the internal value for the desired delta
	* velocity.
	*/
	void calculatedesireddeltavelocity(float duration);

	/**
	* calculates and returns the velocity of the contact
	* point on the given body.
	*/
	_vec3 calculatelocalvelocity(uint32_t bodyindex, float duration);

	/**
	* calculates an orthonormal basis for the contact point, based on
	* the primary friction direction (for anisotropic friction) or
	* a random orientation (for isotropic friction).
	*/
	void calculatecontactbasis();

	/**
	* applies an impulse to the given body, returning the
	* change in velocities.
	*/
	void applyimpulse(const _vec3 &impulse, rigid_body *body,_vec3 *velocitychange, _vec3 *rotationchange);

	/**
	* performs an inertia-weighted impulse based resolution of this
	* contact alone.
	*/
	void applyvelocitychange(_vec3 velocitychange[2],_vec3 rotationchange[2]);

	/**
	* performs an inertia weighted penetration resolution of this
	* contact alone.
	*/
	void applypositionchange(_vec3 linearchange[2], _vec3 angularchange[2], float penetration);

	/**
	* calculates the impulse needed to resolve this contact,
	* given that the contact has no friction. a pair of inertia
	* tensors - one for each contact object - is specified to
	* save calculation time: the calling function has access to
	* these anyway.
	*/
	_vec3 calculatefrictionlessimpulse( _mat3 *inverseinertiatensor);

	/**
	* calculates the impulse needed to resolve this contact,
	* given that the contact has a non-zero coefficient of
	* friction. a pair of inertia tensors - one for each contact
	* object - is specified to save calculation time: the calling
	* function has access to these anyway.
	*/
	_vec3 calculatefrictionimpulse(_mat3 *inverseinertiatensor);
};

/**
* the contact resolution routine. one resolver instance
* can be shared for the whole simulation, as long as you need
* roughly the same parameters each time (which is normal).
*
* resolution algorithm
*
* the resolver uses an iterative satisfaction algorithm; it loops
* through each contact and tries to resolve it. each contact is
* resolved locally, which may in turn put other contacts in a worse
* position. the algorithm then revisits other contacts and repeats
* the process up to a specified iteration limit. it can be proved
* that given enough iterations, the simulation will get to the
* correct result. as with all approaches, numerical stability can
* cause problems that make a correct resolution impossible.
*
* strengths
*
* this algorithm is very fast, much faster than other physics
* approaches. even using many more iterations than there are
* contacts, it will be faster than global approaches.
*
* many global algorithms are unstable under high friction, this
* approach is very robust indeed for high friction and low
* restitution values.
*
* the algorithm produces visually believable behaviour. tradeoffs
* have been made to err on the side of visual realism rather than
* computational expense or numerical accuracy.
*
* weaknesses
*
* the algorithm does not cope well with situations with many
* inter-related contacts: stacked boxes, for example. in this
* case the simulation may appear to jiggle slightly, which often
* dislodges a box from the stack, allowing it to collapse.
*
* another issue with the resolution mechanism is that resolving
* one contact may make another contact move sideways against
* friction, because each contact is handled independently, this
* friction is not taken into account. if one object is pushing
* against another, the pushed object may move across its support
* without friction, even though friction is set between those bodies.
*
* in general this resolver is not suitable for stacks of bodies,
* but is perfect for handling impact, explosive, and flat resting
* situations.
*/

struct contact_resolver {

	/**
	* holds the number of iterations to perform when resolving
	* velocity.
	*/
	uint32_t m_velocity_iterations;

	/**
	* holds the number of iterations to perform when resolving
	* position.
	*/
	uint32_t m_position_iterations;

	/**
	* to avoid instability velocities smaller
	* than this value are considered to be zero. too small and the
	* simulation may be unstable, too large and the bodies may
	* interpenetrate visually. a good starting point is the default
	* of 0.01.
	*/
	float m_velocity_epsilon;

	/**
	* to avoid instability penetrations
	* smaller than this value are considered to be not interpenetrating.
	* too small and the simulation may be unstable, too large and the
	* bodies may interpenetrate visually. a good starting point is
	* the default of0.01.
	*/
	float m_position_epsilon;

	/**
	* stores the number of velocity iterations used in the
	* last call to resolve contacts.
	*/
	uint32_t m_velocity_iterations_used;

	/**
	* stores the number of position iterations used in the
	* last call to resolve contacts.
	*/
	uint32_t m_position_iterations_used;

	/**
	* keeps track of whether the internal settings are valid.
	*/
	bool m_valid_settings;

	/**
	* creates a new contact resolver with the given number of iterations
	* per resolution call, and optional epsilon values.
	*/
	contact_resolver(
		uint32_t iterations,
		float velocityepsilon = 0.01f,
		float positionepsilon = 0.01f
		);

	/**
	* creates a new contact resolver with the given number of iterations
	* for each kind of resolution, and optional epsilon values.
	*/
	contact_resolver(
		uint32_t velocityiterations,
		uint32_t positioniterations,
		float velocityepsilon = 0.01f,
		float positionepsilon = 0.01f
		);

	/**
	* returns true if the resolver has valid settings and is ready to go.
	*/
	bool isvalid() {
		return (m_velocity_iterations > 0) &&
			(m_position_iterations > 0) &&
			(m_position_epsilon >= 0.0f) &&
			(m_position_epsilon >= 0.0f);
	}

	/**
	* sets the number of iterations for each resolution stage.
	*/
	void setiterations(uint32_t velocityiterations, uint32_t positioniterations);

	/**
	* sets the number of iterations for both resolution stages.
	*/
	void setiterations(uint32_t iterations);

	/**
	* sets the tolerance value for both velocity and position.
	*/
	void setepsilon(float velocityepsilon, float positionepsilon);

	/**
	* resolves a set of contacts for both penetration and velocity.
	*
	* contacts that cannot interact with
	* each other should be passed to separate calls to resolvecontacts,
	* as the resolution algorithm takes much longer for lots of
	* contacts than it does for the same number of contacts in small
	* sets.
	*
	*/
	void resolvecontacts(contact *contactarray, uint32_t numcontacts, float duration);

	/**
	* sets up contacts ready for processing. this makes sure their
	* internal data is configured correctly and the correct set of bodies
	* is made alive.
	*/
	void preparecontacts(contact *contactarray, uint32_t numcontacts, float duration);

	/**
	* resolves the velocity issues with the given array of constraints,
	* using the given number of iterations.
	*/
	void adjustvelocities(contact *contactarray, uint32_t numcontacts, float duration);

	/**
	* resolves the positional issues with the given array of constraints,
	* using the given number of iterations.
	*/
	void adjustpositions(contact *contacts,uint32_t numcontacts, float duration);
};

/**
* this is the basic polymorphic interface for contact generators
* applying to rigid bodies.
*/
struct  contact_generator {

	/**
	* fills the given contact structure with the generated
	* contact. the contact pointer should point to the first
	* available contact in a contact array, where limit is the
	* maximum number of contacts in the array that can be written
	* to. the method returns the number of contacts that have
	* been written.
	*/
	virtual uint32_t addcontact(contact *contact, uint32_t limit) const = 0;
};
