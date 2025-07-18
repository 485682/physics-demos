#pragma once

#include "application_header.h"
#include "physics.h"


struct sailing_simulator : public application_object {

	_string m_state_string;

	buoyancy m_buoyancy;

	aero m_sail;
	rigid_body m_sail_boat;
	force_registry m_registry;

	random r;
	_vec3 m_wind_speed;

    float m_sail_control;

	sailing_simulator();

	/**
	* vertex buffer for drawing quads (ground)
	*/
	IDirect3DVertexBuffer9* m_buffer;

	virtual bool init();
	virtual void clear();
	virtual bool update();

	virtual bool render();

	virtual _string  getstatestring() { return m_state_string; }

	virtual void key(uint8_t key){}

};
