#pragma once

#include "application_header.h"
#include "physics.h"


#define NUM_BONES 12
#define NUM_JOINTS 11

struct bone : public collision_box {

	bone() { m_body = new rigid_body(); }

	~bone() { delete m_body; }

	/**
	* we use a sphere to collide bone on bone to allow some limited
	* interpenetration.
	*/
	collision_sphere getcollisionsphere() const {
		
		collision_sphere sphere;
		sphere.m_body = m_body;
		sphere.m_radius = m_half_size.x;
		sphere.m_offset = _mat4();
		if ( m_half_size.y < sphere.m_radius) sphere.m_radius = m_half_size.y;
		if ( m_half_size.z < sphere.m_radius) sphere.m_radius = m_half_size.z;
		sphere.calculateinternals();
		return sphere;
	}

	/** sets the bone to a specific location. */
	void setstate(const _vec3 &position, const _vec3 &extents) {

		m_body->setposition(position);
		m_body->setorientation( _quaternion() );
		m_body->setvelocity( _vec3());
		m_body->setrotation( _vec3());
		m_half_size = extents;

		float mass = m_half_size.x * m_half_size.y * m_half_size.z * 8.0f;
		m_body->setmass(mass);

		_mat3 tensor;
		tensor.setblockinertiatensor(m_half_size, mass);
		m_body->setinertiatensor(tensor);

		m_body->setlineardamping(0.95f);
		m_body->setangulardamping(0.8f);
		m_body->clearaccumulators();
		m_body->setacceleration( _utility::gravity );

		m_body->setcansleep(false);
		m_body->setawake();

		m_body->calculatederiveddata();
		calculateinternals();
	}

};

struct ragdoll : public application_object {

	_string m_state_string;

    /** holds the maximum number of contacts. */
    const static unsigned maxcontacts = 256;

    /** holds the array of contacts. */
    contact m_contacts[maxcontacts];

    /** holds the collision data structure for collision detection. */
    collision_data m_cdata;

    /** holds the contact resolver. */
    contact_resolver m_resolver;

    /** true if the contacts should be rendered. */
    bool m_render_debug_info;

	bool m_pause_simulation;

    /** pauses the simulation after the next frame automatically */
    bool m_auto_pause_simulation;

    /** processes the contact generation code. */
    void generatecontacts();

    /** processes the objects in the simulation forward in time. */
    void updateobjects( float duration);

    void reset();


	random m_random;

    /** Holds the bone bodies. */
    bone bones[NUM_BONES];

    /** Holds the joints. */
    joint m_joints[NUM_JOINTS];

	/**
	* vertex buffers for drawing points lines (ground)
	*/
	IDirect3DVertexBuffer9* m_buffer;
	IDirect3DVertexBuffer9* m_point_buffer;

	ragdoll();

	virtual bool init();
	virtual void clear();
	virtual bool update();

	virtual bool render();

	/**
	* finishes drawing the frame, adding debugging information
	* as needed.
	*/
	bool drawdebug();

	virtual _string  getstatestring() { return m_state_string; }

	virtual void key(uint8_t key);

};
