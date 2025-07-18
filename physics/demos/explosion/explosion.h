#pragma once

#include "application_header.h"
#include "physics.h"

#define OBJECTS 4


struct ball : public collision_sphere {

	ball()  { m_body = new rigid_body; }

	~ball() { delete m_body; }

	/** Sets the box to a specific location. */
	void setstate( _vec3 position, _quaternion orientation, float radius, _vec3 velocity) {
		m_body->setposition(position);
		m_body->setorientation(orientation);
		m_body->setvelocity(velocity);
		m_body->setrotation( _vec3(0,0,0));
		m_radius = radius;

		float mass = 4.0f*0.3333f*3.1415f * radius*radius*radius;
		m_body->setmass(mass);

		_mat3 tensor;
		float coeff = 0.4f*mass*radius*radius;
		tensor.setinertiatensorcoeffs(coeff,coeff,coeff);
		m_body->setinertiatensor(tensor);

		m_body->setlineardamping(0.95f);
		m_body->setangulardamping(0.8f);
		m_body->clearaccumulators();
		m_body->setacceleration(0,-10.0f,0);

		//body->setcansleep(false);
		m_body->setawake();

		m_body->calculatederiveddata();
	}

	/** positions the box at a random location. */
	void random( random *random_) {
		const static _vec3 minpos(-5, 5, -5);
		const static _vec3 maxpos(5, 10, 5);
		//random random_;

		setstate(
			random_->randomvector(minpos, maxpos),
			random_->randomquaternion(),
			random_->randomreal(0.5f, 1.5f),
			_vec3()
			);
	}
};

struct box : public collision_box {

	bool m_is_over_lapping;

	box()  { m_body = new rigid_body; }

	~box() { delete m_body; }

	/** sets the box to a specific location. */
	void setstate(const _vec3 &position,
		const _quaternion &orientation,
		const _vec3 &extents,
		const _vec3 &velocity)
	{
		m_body->setposition(position);
		m_body->setorientation(orientation);
		m_body->setvelocity(velocity);
		m_body->setrotation( _vec3(0,0,0) );
		m_half_size = extents;

		float mass = m_half_size.x * m_half_size.y * m_half_size.z * 8.0f;
		m_body->setmass(mass);

		_mat3 tensor;
		tensor.setblockinertiatensor( m_half_size , mass);
		m_body->setinertiatensor(tensor);

		m_body->setlineardamping(0.95f);
		m_body->setangulardamping(0.8f);
		m_body->clearaccumulators();
		m_body->setacceleration(0,-10.0f,0);

		m_body->setawake();

		m_body->calculatederiveddata();
	}

	/** positions the box at a random location. */
	void random( random *random ) {
		const static _vec3 minpos(-5, 5, -5);
		const static _vec3 maxpos(5, 10, 5);
		const static _vec3 minsize(0.5f, 0.5f, 0.5f);
		const static _vec3 maxsize(4.5f, 1.5f, 1.5f);
		
		setstate(
			random->randomvector(minpos, maxpos),
			random->randomquaternion(),
			random->randomvector(minsize, maxsize),
			_vec3()
			);
	}
};


struct explosion : public application_object {

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
	bool m_edit_mode;
	bool m_up_mode;

	/**
	* holds the number of boxes in the simulation.
	*/
	const static unsigned m_boxes = OBJECTS;

	/** holds the box data. */
	box m_box_data[m_boxes];

	/**
	* holds the number of balls in the simulation.
	*/
	const static unsigned m_balls = OBJECTS;

	/** holds the ball data. */
	ball m_ball_data[m_balls];


	void fire();

	/**
	* vertex buffers for drawing points lines (ground)
	*/
	IDirect3DVertexBuffer9* m_buffer;
	IDirect3DVertexBuffer9* m_point_buffer;

	explosion();

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
