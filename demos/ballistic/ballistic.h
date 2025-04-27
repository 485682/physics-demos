#pragma once

#include "particle.h"

struct ballistic  : public application_object{

	_string m_state_string;

	enum shottype{
		UNUSED = 0,
		PISTOL,
		ARTILLERY,
		FIREBALL,
		LASER
	};
	/**
	* holds a single ammunition round record.
	*/
	struct ammo_round{
		particle m_particle;
		shottype m_type;
		float    m_update_time;
	};

	/**
	* holds the maximum number of  rounds that can be
	* fired.
	*/
	const static unsigned m_ammo_rounds = 16;

	/** 
	* holds the particle data 
	*/
	ammo_round m_ammo[m_ammo_rounds];

	/** 
	* holds the current shot type 
	*/
	shottype   m_current_shot_type;

	/**
	* vertex buffer for drawing lines
	*/
	IDirect3DVertexBuffer9* m_buffer;

	virtual bool init();
	virtual void clear();
	virtual bool update();

	bool render();

	virtual void  leftmouse();
	virtual void  key(uint8_t key);

	virtual _string getstatestring() { return m_state_string; }

};

