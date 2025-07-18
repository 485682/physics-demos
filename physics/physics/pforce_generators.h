#pragma once

#include "particle.h"

/**
* a force generator can be asked to add a force to one or more
* particles.
*/
struct particle_force_generator {

	/**
	* overload this in implementations of the interface to calculate
	* and update the force applied to the given particle.
	*/
	virtual void updateforce(particle *_particle, float duration) = 0;
};

/**
* a force generator that applies a gravitational force. one instance
* can be used for multiple particles.
*/
struct particle_gravity : public particle_force_generator {

	/** holds the acceleration due to gravity. */
	_vec3 m_gravity;

	/** creates the generator with the given acceleration. */
	particle_gravity(const _vec3 &gravity);

	/** applies the gravitational force to the given particle. */
	virtual void updateforce(particle *_particle, float duration);
};

/**
* a force generator that applies a drag force. one instance
* can be used for multiple particles.
*/
struct particle_drag : public particle_force_generator {

	/** holds the velocity drag coeffificent. */
	float m_k1;

	/** holds the velocity squared drag coeffificent. */
	float m_k2;

	/** creates the generator with the given coefficients. */
	particle_drag(float k1, float k2);

	/** applies the drag force to the given particle. */
	virtual void updateforce(particle *_particle, float duration);
};

/**
* a force generator that applies a spring force.
*/
struct particle_spring : public particle_force_generator {
	/** the particle at the other end of the spring. */
	particle * m_other;

	/** holds the sprint constant. */
	float m_spring_constant;

	/** holds the rest length of the spring. */
	float m_rest_length;

	/** creates a new spring with the given parameters. */
	particle_spring(particle *other,float spring_constant, float rest_length);

	/** applies the spring force to the given particle. */
	virtual void updateforce(particle *_particle, float duration);
};

/**
* a force generator that applies a spring force, where
* one end is attached to a fixed point in space.
*/
struct particle_anchored_spring : public particle_force_generator {

	/** the location of the anchored end of the spring. */
	_vec3 *m_anchor;

	/** holds the sprint constant. */
	float m_spring_constant;

	/** holds the rest length of the spring. */
	float m_rest_length;

	particle_anchored_spring();

	/** creates a new spring with the given parameters. */
	particle_anchored_spring(_vec3 *anchor, float springconstant, float restlength);

	/** retrieve the anchor point. */
	const _vec3* getanchor() const { return m_anchor; }

	/** set the spring's properties. */
	void init(_vec3 *anchor,float springconstant,float restlength);

	/** applies the spring force to the given particle. */
	virtual void updateforce(particle *_particle, float duration);
};

/**
* a force generator that fakes a stiff spring force, and where
* one end is attached to a fixed point in space.
*/
struct particle_fake_spring : public particle_force_generator {

	/** the location of the anchored end of the spring. */
	_vec3 *m_anchor;

	/** holds the sprint constant. */
	float m_spring_constant;

	/** holds the damping on the oscillation of the spring. */
	float m_damping;

	/** creates a new spring with the given parameters. */
	particle_fake_spring(_vec3 *anchor, float springConstant,float damping);

	/** applies the spring force to the given particle. */
	virtual void updateforce(particle *_particle, float duration);
};

/**
* a force generator that applies a bungee force, where
* one end is attached to a fixed point in space.
*/
class particle_anchored_bungee : public particle_anchored_spring {

	/** applies the spring force to the given particle. */
	virtual void updateforce(particle *_particle, float duration);
};


/**
* a force generator that applies a spring force only
* when extended.
*/
struct particle_bungee : public particle_force_generator {
	/** the particle at the other end of the spring. */
	particle * m_other;

	/** holds the sprint constant. */
	float m_spring_constant;

	/**
	* holds the length of the bungee at the point it begins to
	* generator a force.
	*/
	float m_rest_length;

	/** creates a new bungee with the given parameters. */
	particle_bungee(particle *other,float springConstant, float restLength);

	/** applies the spring force to the given particle. */
	virtual void updateforce(particle *_particle, float duration);
};

/**
* a force generator that applies a buoyancy force for a plane of
* liquid parrallel to xz plane.
*/
struct particle_buoyancy : public particle_force_generator {
	/**
	* the maximum submersion depth of the object before
	* it generates its maximum boyancy force.
	*/
	float m_max_depth;

	/**
	* the volume of the object
	*/
	float m_volume;

	/**
	* the height of the water plane above y=0. the plane will be
	* parrallel to the xz plane.
	*/
	float m_water_height;

	/**
	* the density of the liquid. pure water has a density of
	* 1000kg per cubic meter.
	*/
	float m_liquid_density;

	/** creates a new buoyancy force with the given parameters. */
	particle_buoyancy(float maxDepth, float volume, float waterHeight,float liquidDensity = 1000.0f);

	/** applies the buoyancy force to the given particle. */
	virtual void updateforce(particle *_particle, float duration);
};

/**
* holds all the force generators and the particles they apply to.
*/
struct particle_force_registry {
	/**
	* keeps track of one force generator and the particle it
	* applies to.
	*/
	struct particle_force_registration{
		particle *m_particle;
		particle_force_generator *m_fg;
	};

	/**
	* holds the list of registrations.
	*/
	typedef _array<particle_force_registration> _registry;
	_registry m_registrations;

	/**
	* registers the given force generator to apply to the
	* given particle.
	*/
	void add(particle* _particle, particle_force_generator *fg);

	/**
	* removes the given registered pair from the registry.
	* if the pair is not registered, this method will have
	* no effect.
	*/
	void remove(particle* _particle, particle_force_generator *fg);

	/**
	* clears all registrations from the registry. this will
	* not delete the particles or the force generators
	* themselves, just the records of their connection.
	*/
	void clear();

	/**
	* calls all the force generators to update the forces of
	* their corresponding particles.
	*/
	void updateforces(float duration);
};