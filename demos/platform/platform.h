#pragma once

#include "application_header.h"
#include "physics.h"

#define rod_count 15

#define base_mass 1
#define extra_mass 10

struct platform : public application_object {

	platform();

	particle_world   m_world;
	particle *       m_particle_array;
	ground_contacts  m_ground_contact_generator;


	_string m_state_string;

	particle_rod * m_rods;

	_vec3 m_mass_position;
	_vec3 m_mass_display_position;

	/**
	* vertex buffer for drawing lines (rods)
	*/
	IDirect3DVertexBuffer9* m_buffer;

	virtual bool init();
	virtual void clear();
	virtual bool update();

	virtual bool render();

	/**
	* Updates particle masses to take into account the mass
	* that's on the platform.
	*/
	void updateadditionalmass();

	virtual _string  getstatestring() { return m_state_string; }

	virtual void key(uint8_t key);
};
