#pragma once

#include "application_header.h"
#include "physics.h"


enum shottype {
    UNUSED = 0,
    PISTOL,
    ARTILLERY,
    FIREBALL,
    LASER
};

struct ammo_round : public collision_sphere {

	shottype m_type;
    float m_update_time;

    ammo_round()  { m_body = new rigid_body; }

    ~ammo_round() { delete m_body; }

    /** sets the box to a specific location. */
    void setstate(shottype shottype) {
        m_type = shottype;

        // set the properties of the particle
        switch(m_type)
        {
        case PISTOL:
            m_body->setmass(1.5f);
            m_body->setvelocity(0.0f, 0.0f, 20.0f);
            m_body->setacceleration(0.0f, -0.5f, 0.0f);
            m_body->setdamping(0.99f, 0.8f);
            m_radius = 0.2f;
            break;

        case ARTILLERY:
            m_body->setmass(200.0f); // 200.0kg
            m_body->setvelocity(0.0f, 30.0f, 40.0f); // 50m/s
            m_body->setacceleration(0.0f, -21.0f, 0.0f);
            m_body->setdamping(0.99f, 0.8f);
            m_radius = 0.4f;
            break;

        case FIREBALL:
            m_body->setmass(4.0f); // 4.0kg - mostly blast damage
            m_body->setvelocity(0.0f, -0.5f, 10.0); // 10m/s
            m_body->setacceleration(0.0f, 0.3f, 0.0f); // floats up
            m_body->setdamping(0.9f, 0.8f);
            m_radius = 0.45f;
            break;

        case LASER:
            // note that this is the kind of laser bolt seen in films,
            // not a realistic laser beam!
            m_body->setmass(0.1f); // 0.1kg - almost no weight
            m_body->setvelocity(0.0f, 0.0f, 100.0f); // 100m/s
            m_body->setacceleration(0.0f, 0.0f, 0.0f); // no gravity
            m_body->setdamping(0.99f, 0.8f);
            m_radius = 0.2f;
            break;
        }

        m_body->setcansleep(false);
        m_body->setawake();

        _mat3 tensor;
        float coeff = 0.4f*m_body->getmass()*m_radius*m_radius;
        tensor.setinertiatensorcoeffs(coeff,coeff,coeff);
        m_body->setinertiatensor(tensor);

        // set the data common to all particle types
        m_body->setposition(0.0f, 1.5f, 0.0f);
        m_update_time = 0;/* count in seconds*/

        // clear the force accumulators
        m_body->calculatederiveddata();
        calculateinternals();
    }
};

struct box : public collision_box {

	box()  { m_body = new rigid_body; }

    ~box() { delete m_body; }

    /** sets the box to a specific location. */
    void setstate( float z) {

        m_body->setposition(0, 3, z);
        m_body->setorientation(1,0,0,0);
        m_body->setvelocity(0,0,0);
        m_body->setrotation( _vec3(0,0,0) );
        m_half_size = _vec3(1,1,1);

        float mass = m_half_size.x * m_half_size.y * m_half_size.z * 8.0f;
        m_body->setmass(mass);

        _mat3 tensor;
        tensor.setblockinertiatensor(m_half_size, mass);
        m_body->setinertiatensor(tensor);

        m_body->setlineardamping(0.95f);
        m_body->setangulardamping(0.8f);
        m_body->clearaccumulators();
        m_body->setacceleration(0,-10.0f,0);

        m_body->setcansleep(false);
        m_body->setawake();

        m_body->calculatederiveddata();
        calculateinternals();
    }
};



struct bigballistic : public application_object {

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

	/**
	* finishes drawing the frame, adding debugging information
	* as needed.
	*/
	void drawdebug();

	void reset();


    /**
     * holds the maximum number of  rounds that can be
     * fired.
     */
    const static unsigned m_ammo_rounds = 256;

    /** 
	* holds the particle data 
	*/
    ammo_round m_ammo[m_ammo_rounds];

    /**
    * holds the number of boxes in the simulation.
    */
    const static unsigned m_boxes = 2;

    /** 
	* holds the box data 
	*/
    box m_box_data[m_boxes];

    /** 
	* holds the current shot type 
	*/
    shottype m_current_shot_type;

	/**
	* vertex buffer for drawing lines
	*/
	IDirect3DVertexBuffer9* m_buffer;

	bigballistic();


	virtual bool init();
	virtual void clear();
	virtual bool update();

	virtual bool render();

	virtual _string  getstatestring() { return m_state_string; }

	virtual void leftmouse();
	virtual void key(uint8_t key);

};
