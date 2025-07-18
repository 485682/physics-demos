
#include "body.h"


/**
* a force generator can be asked to add a force to one or more
* bodies.
*/
struct force_generator {
	/**
	* overload this in implementations of the interface to calculate
	* and update the force applied to the given rigid body.
	*/
	virtual void updateforce(rigid_body *body, float duration) = 0;
};

/**
* a force generator that applies a gravitational force. one instance
* can be used for multiple rigid bodies.
*/
struct gravity : public force_generator {
	/** holds the acceleration due to gravity. */
	_vec3 m_gravity;
	/** creates the generator with the given acceleration. */
	gravity(const _vec3 &gravity);

	/** applies the gravitational force to the given rigid body. */
	virtual void updateforce(rigid_body *body, float duration);
};

/**
* a force generator that applies a spring force.
*/
struct spring : public force_generator {
	/**
	* the point of connection of the spring, in local
	* coordinates.
	*/
	_vec3 m_connection_point;

	/**
	* the point of connection of the spring to the other object,
	* in that object's local coordinates.
	*/
	_vec3 m_other_connection_point;

	/** the particle at the other end of the spring. */
	rigid_body *m_other;

	/** holds the sprint constant. */
	float m_spring_constant;

	/** holds the rest length of the spring. */
	float m_rest_length;

	/** creates a new spring with the given parameters. */
	spring(const _vec3 &localConnectionPt,
		rigid_body *other,
		const _vec3 &otherConnectionPt,
		float springConstant,
		float restLength);

	/** applies the spring force to the given rigid body. */
	virtual void updateforce(rigid_body *body, float duration);
};


/**
* a force generator that applies an aerodynamic force.
*/
struct aero : public force_generator {
	/**
	* holds the aerodynamic tensor for the surface in body
	* space.
	*/
	_mat3 m_tensor;

	/**
	* holds the relative position of the aerodynamic surface in
	* body coordinates.
	*/
	_vec3 m_position;

	/**
	* holds a pointer to a vector containing the windspeed of the
	* environment. this is easier than managing a separate
	* windspeed vector per generator and having to update it
	* manually as the wind changes.
	*/
	const _vec3* m_wind_speed;

	/**
	* creates a new aerodynamic force generator with the
	* given properties.
	*/
	aero(const _mat3 &tensor, const _vec3 &position,const _vec3 *windspeed);

	/**
	* applies the force to the given rigid body.
	*/
	virtual void updateforce(rigid_body *body, float duration);

	/**
	* uses an explicit tensor matrix to update the force on
	* the given rigid body. this is exactly the same as for updateforce
	* only it takes an explicit tensor.
	*/
	void updateforcefromtensor(rigid_body *body, float duration,const _mat3 &tensor);
};

/**
* A force generator with a control aerodynamic surface. This
* requires three inertia tensors, for the two extremes and
* 'resting' position of the control surface.  The latter tensor is
* the one inherited from the base class, the two extremes are
* defined in this class.
*/
struct aero_control : public aero {

	/**
	* The aerodynamic tensor for the surface, when the control is at
	* its maximum value.
	*/
	_mat3 m_max_tensor;

	/**
	* The aerodynamic tensor for the surface, when the control is at
	* its minimum value.
	*/
	_mat3 m_min_tensor;

	/**
	* The current position of the control for this surface. This
	* should range between -1 (in which case the minTensor value
	* is used), through 0 (where the base-class tensor value is
	* used) to +1 (where the maxTensor value is used).
	*/
	float m_control_setting;

	/**
	* Calculates the final aerodynamic tensor for the current
	* control setting.
	*/
	_mat3 gettensor();

	/**
	* Creates a new aerodynamic control surface with the given
	* properties.
	*/
	aero_control(const _mat3 &base, const _mat3 &min, const _mat3 &max,const _vec3 &position, const _vec3 *windspeed);

	/**
	* Sets the control position of this control. This * should
	range between -1 (in which case the minTensor value is *
	used), through 0 (where the base-class tensor value is used) *
	to +1 (where the maxTensor value is used). Values outside that
	* range give undefined results.
	*/
	void setcontrol(float value);

	/**
	* Applies the force to the given rigid body.
	*/
	virtual void updateforce(rigid_body *body, float duration);
};

/**
* a force generator with an aerodynamic surface that can be
* re-oriented relative to its rigid body. 
*/
struct angled_aero : public aero {
	/**
	* holds the orientation of the aerodynamic surface relative
	* to the rigid body to which it is attached.
	*/
	_quaternion m_orientation;

	/**
	* creates a new aerodynamic surface with the given properties.
	*/
	angled_aero(const _mat3 &tensor, const _vec3 &position,const _vec3 *windspeed);

	/**
	* sets the relative orientation of the aerodynamic surface,
	* relative to the rigid body it is attached to. note that
	* this doesn't affect the point of connection of the surface
	* to the body.
	*/
	void setorientation(const _quaternion &quat);

	/**
	* applies the force to the given rigid body.
	*/
	virtual void updateforce(rigid_body *body, float duration);
};

/**
* a force generator to apply a buoyant force to a rigid body.
*/
struct buoyancy : public force_generator {
	/**
	* the maximum submersion depth of the object before
	* it generates its maximum buoyancy force.
	*/
	float m_max_depth;

	/**
	* the volume of the object.
	*/
	float m_volume;

	/**
	* the height of the water plane above y=0. the plane will be
	* parallel to the xz plane.
	*/
	float m_water_height;

	/**
	* the density of the liquid. pure water has a density of
	* 1000kg per cubic meter.
	*/
	float m_liquid_density;

	/**
	* the centre of buoyancy of the rigid body, in body coordinates.
	*/
	_vec3 m_centre_of_buoyancy;

	/** creates a new buoyancy force with the given parameters. */
	buoyancy(const _vec3 &cofb,float maxdepth, float volume, float waterheight,float liquiddensity = 1000.0f);

	/**
	* applies the force to the given rigid body.
	*/
	virtual void updateforce(rigid_body *body, float duration);
};

/**
* holds all the force generators and the bodies they apply to.
*/
struct force_registry {

	/**
	* keeps track of one force generator and the body it
	* applies to.
	*/
	struct force_registration {
		rigid_body *m_body;
		force_generator *m_fg;
	};

	/**
	* holds the list of registrations.
	*/
	typedef _array<force_registration> _registry;
	_registry m_registrations;

	/**
	* registers the given force generator to apply to the
	* given body.
	*/
	void add(rigid_body* body, force_generator *fg);

	/**
	* removes the given registered pair from the registry.
	* if the pair is not registered, this method will have
	* no effect.
	*/
	void remove(rigid_body* body, force_generator *fg);

	/**
	* clears all registrations from the registry. this will
	* not delete the bodies or the force generators
	* themselves, just the records of their connection.
	*/
	void clear();

	/**
	* calls all the force generators to update the forces of
	* their corresponding bodies.
	*/
	void updateforces(float duration);
};
