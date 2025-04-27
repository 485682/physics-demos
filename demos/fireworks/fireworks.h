#pragma once

#include "application_header.h"

#include "random.h"
#include "particle.h"

static random crandom;

/**
* fireworks are particles, with additional data for rendering and
* evolution.
*/
struct firework : public particle {

	/** fireworks have an integer type, used for firework rules. */
	uint32_t m_type;

	/**
	* the age of a firework determines when it detonates. age gradually
	* decreases, when it passes zero the firework delivers its payload.
	* think of age as fuse-left.
	*/
	float m_age;

	/**
	* updates the firework by the given duration of time. returns true
	* if the firework has reached the end of its life and needs to be
	* removed.
	*/
	bool update(float duration) {
		// update our physical state
		integrate(duration);

		// we work backwards from our age to zero.
		m_age -= duration;
		return (m_age < 0) || (m_position.y < 0);
	}
};

/**
* firework rules control the length of a firework's fuse and the
* particles it should evolve into.
*/
struct firework_rule {

	/** the type of firework that is managed by this rule. */
	uint32_t m_type;

	/** the minimum length of the fuse. */
	float    m_min_age;

	/** the maximum legnth of the fuse. */
	float    m_max_age;

	/** the minimum relative velocity of this firework. */
	_vec3   m_min_velocity;

	/** the maximum relative velocity of this firework. */
	_vec3   m_max_velocity;

	/** the damping of this firework type. */
	float   m_damping;

	/**
	* the payload is the new firework type to create when this
	* firework's fuse is over.
	*/
	struct payload {
		/** the type of the new particle to create. */
		uint32_t m_type;

		/** the number of particles in this payload. */
		uint32_t m_count;

		/** sets the payload properties in one go. */
		void set(uint32_t type, uint32_t count){
			m_type = type;
			m_count = count;
		}
	};

	/** the number of payloads for this firework type. */
	uint32_t m_payload_count;

	/** the set of payloads. */
	payload *m_payloads;

	firework_rule() : m_payload_count(0),  m_payloads(NULL) { }

	void init(uint32_t payloadcount) {
		firework_rule::m_payload_count = payloadcount;
		m_payloads = new payload[payloadcount];
	}

	~firework_rule() { if (m_payloads != NULL) delete[] m_payloads; }

	/**
	* set all the rule parameters in one go.
	*/
	void setParameters(uint32_t type, float minage, float maxage,
		const _vec3 &minvelocity, const _vec3 &maxvelocity,
		float damping)
	{
		m_type         = type;
		m_min_age      = minage;
		m_max_age      = maxage;
		m_min_velocity = minvelocity;
		m_max_velocity = maxvelocity;
		m_damping      = damping;
	}

	/**
	* creates a new firework of this type and writes it into the given
	* instance. the optional parent firework is used to base position
	* and velocity on.
	*/
	void create(firework *_firework, const firework *parent = NULL) const
	{
		_firework->m_type = m_type;
		_firework->m_age = crandom.randomreal(m_min_age, m_max_age);

		_vec3 vel;
		if (parent) {
			// the position and velocity are based on the parent.
			_firework->setposition(parent->getposition());
			vel += parent->getvelocity();
		}else{
			_vec3 start;
			int x = (int)crandom.randomint(3) - 1;
			start.x = 5.0f * float(x);
			_firework->setposition(start);
		}

		vel += crandom.randomvector(m_min_velocity, m_max_velocity);
		_firework->setvelocity(vel);

		// we use a mass of one in all cases (no point having fireworks
		// with different masses, since they are only under the influence
		// of gravity).
		_firework->setmass(1);

		_firework->setdamping(m_damping);

		_firework->setacceleration(_utility::gravity);

		_firework->clearaccumulator();
	}
};


struct fireworks  : public application_object{

	 _string m_state_string;

	/**
     * holds the maximum number of fireworks that can be in use.
     */
    const static uint32_t max_fireworks = 1024;

    /** holds the firework data. */
    firework m_firework_buffer[max_fireworks];

    /** holds the index of the next firework slot to use. */
    uint32_t m_next_firework;

    /** and the number of rules. */
    const static uint32_t rule_count = 9;

    /** holds the set of rules. */
    firework_rule rules[rule_count];

    /** dispatches a firework from the origin. */
    void create(uint32_t type, const firework *parent=NULL);

    /** dispatches the given number of fireworks from the given parent. */
    void create(uint32_t type, uint32_t number, const firework *parent);

    /** creates the rules. */
    void initfireworkrules();

	/**
	* vertex buffer for drawing quads ( fireworks )
	*/
	IDirect3DVertexBuffer9* m_buffer;

	virtual bool init();
	virtual void clear();
	virtual bool update();

	bool render();

	virtual _string  getstatestring() { return m_state_string; }

	virtual void key(uint8_t key);

};

