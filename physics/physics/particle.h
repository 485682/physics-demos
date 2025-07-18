#pragma once

#include "application_header.h"

/*
* a particle is the simplest object that can be simulated in the
* physics system.
*
* it has position data (no orientation data), along with
* velocity. it can be integrated forward through time, and have
* linear forces, and impulses applied to it. the particle manages
* its state and allows access through a set of methods.
*/

struct  particle {

	/**
	* holds the inverse of the mass of the particle. it
	* is more useful to hold the inverse mass because
	* integration is simpler, and because in real time
	* simulation it is more useful to have objects with
	* infinite mass (immovable) than zero mass
	* (completely unstable in numerical simulation).
	*/
	float m_inverse_mass;

	/**
	* holds the amount of damping applied to linear
	* motion. damping is required to remove energy added
	* through numerical instability in the integrator.
	*/
	float m_damping;

	/**
	* holds the linear position of the particle in
	* world space.
	*/
	_vec3 m_position;

	/**
	* holds the linear velocity of the particle in
	* world space.
	*/
	_vec3 m_velocity;

	/**
	* holds the accumulated force to be applied at the next
	* simulation iteration only. this value is zeroed at each
	* integration step.
	*/
	_vec3 m_forces;

	/**
	* holds the acceleration of the particle.  this value
	* can be used to set acceleration due to gravity (its primary
	* use), or any other constant acceleration.
	*/
	_vec3 m_acceleration;

	/**
	* integrates the particle forward in time by the given amount.
	* this function uses a newton-euler integration method, which is a
	* linear approximation to the correct integral. for this reason it
	* may be inaccurate in some cases.
	*/
	void  integrate(float duration);

	/**
	* sets the mass of the particle
	*/
	void  setmass(const float mass);

	/**
	* gets the mass of the particle
	*/
	float getmass() const;

	/**
	* sets the inverse mass of the particle
	*/
	void  setinversemass(const float inverse_mass);

	/**
	* gets the inverse mass of the particle
	*/
	float getinversemass() const;

	/**
	* returns true if the mass of the particle is not-infinite
	*/
	bool  hasfinitemass() const;

	/**
	* sets the damping of the particle
	*/
	void  setdamping(const float damping);

	/**
	* gets the current damping value
	*/
	float getdamping() const;

	/**
	* sets the position of the particle
	*/
	void  setposition(const _vec3 &position);

	/**
	* sets the position of the particle by component
	*/
	void  setposition(const float x, const float y, const float z);

	/**
	* fills the given vector with the position of the particle
	*/
	void  getposition(_vec3 *position) const;

	/**
	* gets the position of the particle
	*/
	_vec3 getposition() const;

	/**
	* sets the velocity of the particle
	*/
	void setvelocity(const _vec3 &velocity);

	/**
	* sets the velocity of the particle by component
	*/
	void setvelocity(const float x, const float y, const float z);

	/**
	* fills the given vector with the velocity of the particle
	*/
	void getvelocity(_vec3 *velocity) const;

	/**
	* gets the velocity of the particle
	*/
	_vec3 getvelocity() const;

	/**
	* sets the constant acceleration of the particle
	*/
	void setacceleration(const _vec3 &acceleration);

	/**
	* sets the constant acceleration of the particle by component
	*/
	void setacceleration(const float x, const float y, const float z);

	/**
	* fills the given vector with the acceleration of the particle
	*/
	void getacceleration(_vec3 *acceleration) const;

	/**
	* gets the acceleration of the particle
	*/
	_vec3 getacceleration() const;

	/**
	* clears the forces applied to the particle. this will be
	* called automatically after each integration step.
	*/
	void clearaccumulator();

	/**
	* adds the given force to the particle, to be applied at the
	* next iteration only.
	*/
	void addforce(const _vec3 &force);

};