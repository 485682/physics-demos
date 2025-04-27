#pragma once

#include "application_header.h"
#include "physics.h"


struct flight_simulator : public application_object {

	_string m_state_string;
	aero_control m_left_wing;
	aero_control m_right_wing;
	aero_control m_rudder;
	aero m_tail;
	rigid_body m_aircraft;
	force_registry m_registry;

    _vec3 m_wind_speed;

    float m_left_wing_control;
    float m_right_wing_control;
    float m_rudder_control;

    void resetplane();

	flight_simulator();


	/**
	* vertex buffer for drawing quads (ground plane)
	*/
	IDirect3DVertexBuffer9* m_buffer;

	virtual bool init();
	virtual void clear();
	virtual bool update();

	virtual bool render();

	virtual _string  getstatestring() { return m_state_string; }

	virtual void key(uint8_t key);

};
