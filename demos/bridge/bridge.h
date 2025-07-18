#pragma once

#include "application_header.h"
#include "physics.h"

#define rod_count     6
#define cable_count   10
#define support_count 12

#define base_mass     1
#define extra_mass    10


struct bridge : public application_object {

	bridge();

	_string m_state_string;

    particle_cable * m_cables;
    particle_rod *   m_rods;
	particle_cable_constraint * m_supports;

	particle_world  m_world;
    particle *      m_particle_array;
    ground_contacts m_ground_contact_generator;


    _vec3 m_mass_pos;
    _vec3 m_mass_display_pos;

	/**
	* vertex buffer for drawing lines ( rods, cables, supports)
	*/
	IDirect3DVertexBuffer9* m_buffer;

	virtual bool init();
	virtual void clear();
	virtual bool update();

	bool render();

    /**
     * updates particle masses to take into account the mass
     * that's crossing the bridge.
     */
	void updateadditionalmass();


	virtual void  key(uint8_t key);

	virtual _string  getstatestring() { return m_state_string; }

};
