#pragma once

#include "application_header.h"


/**
* a rigid body is the basic simulation object in the physics
* core.
*
* it has position and orientation data, along with first
* derivatives. it can be integrated forward through time, and
* have forces, torques and impulses (linear or angular) applied
* to it. the rigid body manages its state and allows access
* through a set of methods.
*
* a ridid body contains 64 words (the size of which is given
* by the precision: sizeof(real)). it contains no virtual
* functions, so should take up exactly 64 words in memory. of
* this total 15 words are padding, distributed among the
* vector3 data members.
*/


struct  rigid_body {

	/**
	*
	* this data holds the state of the rigid body. there are two
	* sets of data: characteristics and state.
	*
	* characteristics are properties of the rigid body
	* independent of its current kinematic situation. this
	* includes mass, moment of inertia and damping
	* properties. two identical rigid bodys will have the same
	* values for their characteristics.
	*
	* state includes all the characteristics and also includes
	* the kinematic situation of the rigid body in the current
	* simulation. by setting the whole state data, a rigid body's
	* exact game state can be replicated. note that state does
	* not include any forces applied to the body. two identical
	* rigid bodies in the same simulation will not share the same
	* state values.
	*
	* the state values make up the smallest set of independent
	* data for the rigid body. other state data is calculated
	* from their current values. when state data is changed the
	* dependent values need to be updated: this can be achieved
	* either by integrating the simulation, or by calling the
	* calculateinternals function. this two stage process is used
	* because recalculating internals can be a costly process:
	* all state changes should be carried out at the same time,
	* allowing for a single call.
	*
	*/

	/**
	* holds the inverse of the mass of the rigid body. it
	* is more useful to hold the inverse mass because
	* integration is simpler, and because in real time
	* simulation it is more useful to have bodies with
	* infinite mass (immovable) than zero mass
	* (completely unstable in numerical simulation).
	*/
	float m_inverse_mass;

	/**
	* holds the inverse of the body's inertia tensor. the
	* inertia tensor provided must not be degenerate
	* (that would mean the body had zero inertia for
	* spinning along one axis). as long as the tensor is
	* finite, it will be invertible. the inverse tensor
	* is used for similar reasons to the use of inverse
	* mass.
	*
	* the inertia tensor, unlike the other variables that
	* define a rigid body, is given in body space.
	*
	*/
	_mat3 m_inverse_inertia_tensor;

	/**
	* holds the amount of damping applied to linear
	* motion.  damping is required to remove energy added
	* through numerical instability in the integrator.
	*/
	float m_linear_damping;

	/**
	* holds the amount of damping applied to angular
	* motion.  damping is required to remove energy added
	* through numerical instability in the integrator.
	*/
	float m_angular_damping;

	/**
	* holds the linear position of the rigid body in
	* world space.
	*/
	_vec3 m_position;

	/**
	* holds the angular orientation of the rigid body in
	* world space.
	*/
	_quaternion m_orientation;

	/**
	* holds the linear velocity of the rigid body in
	* world space.
	*/
	_vec3 m_velocity;

	/**
	* holds the angular velocity, or rotation, or the
	* rigid body in world space.
	*/
	_vec3 m_rotation;


	/**
	* these data members hold information that is derived from
	* the other data in the class.
	*/

	/**
	* holds the inverse inertia tensor of the body in world
	* space. the inverse inertia tensor member is specified in
	* the body's local space.
	*
	*/
	_mat3 m_inverse_inertia_tensor_world;

	/**
	* holds the amount of motion of the body. this is a recency
	* weighted mean that can be used to put a body to sleap.
	*/
	float m_motion;

	/**
	* a body can be put to sleep to avoid it being updated
	* by the integration functions or affected by collisions
	* with the world.
	*/
	bool m_isawake;

	/**
	* some bodies may never be allowed to fall asleep.
	* user controlled bodies, for example, should be
	* always awake.
	*/
	bool m_cansleep;

	/**
	* holds a transform matrix for converting body space into
	* world space and vice versa. this can be achieved by calling
	* the getpointin*space functions.
	*
	*/
	_mat4 m_transform_matrix;


	/**
	*
	* these data members store the current force, torque and
	* acceleration of the rigid body. forces can be added to the
	* rigid body in any order, and the class decomposes them into
	* their constituents, accumulating them for the next
	* simulation step. at the simulation step, the accelerations
	* are calculated and stored to be applied to the rigid body.
	*/

	/**
	* holds the accumulated force to be applied at the next
	* integration step.
	*/
	_vec3 m_force_accumulated;

	/**
	* holds the accumulated torque to be applied at the next
	* integration step.
	*/
	_vec3 m_torque_accumulated;

	/**
	* holds the acceleration of the rigid body.  this value
	* can be used to set acceleration due to gravity (its primary
	* use), or any other constant acceleration.
	*/
	_vec3 m_acceleration;

	/**
	* holds the linear acceleration of the rigid body, for the
	* previous frame.
	*/
	_vec3 m_last_frame_acceleration;

	/**
	*
	* there are no data members in the rigid body class that are
	* created on the heap. so all data storage is handled
	* automatically.
	*/


	/**
	*
	* these functions are used to simulate the rigid body's
	* motion over time. a normal application sets up one or more
	* rigid bodies, applies permanent forces (i.e. gravity), then
	* adds transient forces each frame, and integrates, prior to
	* rendering.
	*
	* currently the only integration function provided is the
	* first order newton euler method.
	*/

	/**
	* calculates internal data from state data. this should be called
	* after the body's state is altered directly (it is called
	* automatically during integration). if you change the body's state
	* and then intend to integrate before querying any data (such as
	* the transform matrix), then you can ommit this step.
	*/
	void calculatederiveddata();

	/**
	* integrates the rigid body forward in time by the given amount.
	* this function uses a newton-euler integration method, which is a
	* linear approximation to the correct integral. for this reason it
	* may be inaccurate in some cases.
	*/
	void integrate(float duration);


	/**
	* these functions provide access to the rigid body's
	* characteristics or state. these data can be accessed
	* individually, or en masse as an array of values
	* (e.g. getcharacteristics, getstate). when setting new data,
	* make sure the calculateinternals function, or an
	* integration routine, is called before trying to get data
	* from the body, since the class contains a number of
	* dependent values that will need recalculating.
	*/

	/**
	* sets the mass of the rigid body.
	*
	* warning this invalidates internal data for the rigid body.
	* either an integration function, or the calculateinternals
	* function should be called before trying to get any settings
	* from the rigid body.
	*/
	void setmass(const float mass);

	/**
	* gets the mass of the rigid body
	*
	*/
	float getmass() const;

	/**
	* sets the inverse mass of the rigid body.
	*
	* warning this invalidates internal data for the rigid body.
	* either an integration function, or the calculateinternals
	* function should be called before trying to get any settings
	* from the rigid body.
	*/
	void setinversemass(const float inversemass);

	/**
	* gets the inverse mass of the rigid body.
	*
	*/
	float getinversemass() const;

	/**
	* returns true if the mass of the body is not-infinite
	*/
	bool hasfinitemass() const;

	/**
	* sets the intertia tensor for the rigid body.
	*
	*
	* warning this invalidates internal data for the rigid body.
	* either an integration function, or the calculateinternals
	* function should be called before trying to get any settings
	* from the rigid body.
	*/
	void setinertiatensor(const _mat3 &inertiatensor);

	/**
	* copies the current inertia tensor of the rigid body into
	* the given matrix.
	*
	*/
	void getinertiatensor(_mat3 *inertiatensor) const;

	/**
	* gets a copy of the current inertia tensor of the rigid body
	*/
	_mat3 getinertiatensor() const;

	/**
	* copies the current inertia tensor of the rigid body into
	* the given matrix.
	*
	*/
	void getinertiatensorworld(_mat3 *inertiatensor) const;

	/**
	* gets a copy of the current inertia tensor of the rigid body.
	*
	*/
	_mat3 getinertiatensorworld() const;

	/**
	* sets the inverse intertia tensor for the rigid body.
	*
	* warning this invalidates internal data for the rigid body.
	* either an integration function, or the calculateinternals
	* function should be called before trying to get any settings
	* from the rigid body.
	*/
	void setinverseinertiatensor(const _mat3 &inverseInertiaTensor);

	/**
	* copies the current inverse inertia tensor of the rigid body
	* into the given matrix.
	*
	*/
	void getinverseinertiatensor(_mat3 *inverseinertiatensor) const;

	/**
	* gets a copy of the current inverse inertia tensor of the
	* rigid body.
	*
	*/
	_mat3 getinverseinertiatensor() const;

	/**
	* copies the current inverse inertia tensor of the rigid body
	* into the given matrix.
	*
	*/
	void getinverseinertiatensorworld(_mat3 *inverseinertiatensor) const;

	/**
	* gets a copy of the current inverse inertia tensor of the
	* rigid body.
	*/
	_mat3 getinverseinertiatensorworld() const;

	/**
	* sets both linear and angular damping in one function call.
	*/
	void setdamping(const float lineardamping, const float angulardamping);

	/**
	* sets the linear damping for the rigid body.
	*/
	void setlineardamping(const float lineardamping);

	/**
	* gets the current linear damping value.
	*/
	float getlineardamping() const;

	/**
	* sets the angular damping for the rigid body.
	*/
	void setangulardamping(const float angulardamping);

	/**
	* gets the current angular damping value.
	*/
	float getangulardamping() const;

	/**
	* sets the position of the rigid body.
	*/
	void setposition(const _vec3 &position);

	/**
	* sets the position of the rigid body by component.
	*/
	void setposition(const float x, const float y, const float z);

	/**
	* fills the given vector with the position of the rigid body.
	*/
	void getposition(_vec3 *position) const;

	/**
	* gets the position of the rigid body.
	*/
	_vec3 getposition() const;

	/**
	* sets the orientation of the rigid body.
	*
	* the given orientation does not need to be normalised,
	* and can be zero. this function automatically constructs a
	* valid rotation quaternion with (0,0,0,0) mapping to
	* (1,0,0,0).
	*/
	void setorientation(const _quaternion &orientation);

	/**
	* sets the orientation of the rigid body by component.
	*
	* the given orientation does not need to be normalised,
	* and can be zero. this function automatically constructs a
	* valid rotation quaternion with (0,0,0,0) mapping to
	* (1,0,0,0).
	*/
	void setorientation(const float r, const float i,const float j, const float k);

	/**
	* fills the given quaternion with the current value of the
	* rigid body's orientation.
	*
	*/
	void getorientation(_quaternion *orientation) const;

	/**
	* gets the orientation of the rigid body.
	*/
	_quaternion getorientation() const;

	/**
	* fills the given matrix with a transformation representing
	* the rigid body's orientation.
	*
	* transforming a direction vector by this matrix turns
	* it from the body's local space to world space.
	*/
	void getorientation(_mat3 *matrix) const;

	/**
	* fills the given matrix data structure with a transformation
	* representing the rigid body's orientation.
	*
	* transforming a direction vector by this matrix turns
	* it from the body's local space to world space.
	*/
	void getorientation(float matrix[9]) const;

	/**
	* fills the given matrix with a transformation representing
	* the rigid body's position and orientation.
	*
	* transforming a vector by this matrix turns it from
	* the body's local space to world space.
	*
	* transform a pointer to the matrix to fill.
	*/
	void gettransform(_mat4 *transform) const;

	/**
	* fills the given matrix data structure with a
	* transformation representing the rigid body's position and
	* orientation.
	*
	* transforming a vector by this matrix turns it from
	* the body's local space to world space.
	*/
	void gettransform(float matrix[16]) const;

	/**
	* gets a transformation representing the rigid body's
	* position and orientation.
	*
	* transforming a vector by this matrix turns it from
	* the body's local space to world space.
	*
	*/
	_mat4 gettransform() const;

	/**
	* converts the given point from world space into the body's
	* local space.
	*/
	_vec3 getpointinlocalspace(const _vec3 &point) const;

	/**
	* converts the given point from world space into the body's
	* local space.
	*/
	_vec3 getpointinworldspace(const _vec3 &point) const;

	/**
	* converts the given direction from world space into the
	* body's local space.
	*
	* when a direction is converted between frames of
	* reference, there is no translation required.
	*
	*/
	_vec3 getdirectioninlocalspace(const _vec3 &direction) const;

	/**
	* converts the given direction from world space into the
	* body's local space.
	*
	* when a direction is converted between frames of
	* reference, there is no translation required.
	*
	*/
	_vec3 getdirectioninworldspace(const _vec3 &direction) const;

	/**
	* sets the velocity of the rigid body.
	* velocity is given in world space.
	*/
	void setvelocity(const _vec3 &velocity);

	/**
	* sets the velocity of the rigid body by component. the
	* velocity is given in world space.
	*/
	void setvelocity(const float x, const float y, const float z);

	/**
	* fills the given vector with the velocity of the rigid body
	*/
	void getvelocity(_vec3 *velocity) const;

	/**
	* gets the velocity of the rigid body.
	*
	* the velocity of the rigid body. the velocity is
	* given in world local space.
	*/
	_vec3 getvelocity() const;

	/**
	* applies the given change in velocity.
	*/
	void addvelocity(const _vec3 &deltavelocity);

	/**
	* sets the rotation of the rigid body.
	*/
	void setrotation(const _vec3 &rotation);

	/**
	* sets the rotation of the rigid body by component. the
	* rotation is given in world space.
	*/
	void setrotation(const float x, const float y, const float z);

	/**
	* fills the given vector with the rotation of the rigid body
	*/
	void getrotation(_vec3 *rotation) const;

	/**
	* gets the rotation of the rigid body
	*
	*/
	_vec3 getrotation() const;

	/**
	* applies the given change in rotation.
	*/
	void addrotation(const _vec3 &deltarotation);

	/**
	* returns true if the body is awake and responding to
	* integration.
	*
	*/
	bool getawake() const { return m_isawake; }

	/**
	* sets the awake state of the body. if the body is set to be
	* not awake, then its velocities are also cancelled, since
	* a moving body that is not awake can cause problems in the
	* simulation.
	*/
	void setawake(const bool awake=true);

	/**
	* returns true if the body is allowed to go to sleep at
	* any time.
	*/
	bool getcansleep() const { return m_cansleep; }

	/**
	* sets whether the body is ever allowed to go to sleep. bodies
	* under the player's control, or for which the set of
	* transient forces applied each frame are not predictable,
	* should be kept awake.
	*
	*/
	void setcansleep(const bool cansleep=true);


	/**
	*
	* these functions provide access to the acceleration
	* properties of the body. the acceleration is generated by
	* the simulation from the forces and torques applied to the
	* rigid body. acceleration cannot be directly influenced, it
	* is set during integration, and represent the acceleration
	* experienced by the body of the previous simulation step.
	*/

	/**
	* fills the given vector with the current accumulated value
	* for linear acceleration. the acceleration accumulators
	* are set during the integration step. they can be read to
	* determine the rigid body's acceleration over the last
	* integration step. the linear acceleration is given in world
	* space.
	*
	*/
	void getlastframeacceleration(_vec3 *linearacceleration) const;

	/**
	* gets the current accumulated value for linear
	* acceleration. the acceleration accumulators are set during
	* the integration step. they can be read to determine the
	* rigid body's acceleration over the last integration
	* step. the linear acceleration is given in world space
	*
	*/
	_vec3 getlastframeacceleration() const;


	/**
	*
	* these functions set up forces and torques to apply to the
	* rigid body.
	*/

	/**
	* clears the forces and torques in the accumulators. this will
	* be called automatically after each intergration step
	*/
	void clearaccumulators();

	/**
	* adds the given force to centre of mass of the rigid body
	* the force is expressed in world-coordinates
	*/
	void addforce(const _vec3 &force);

	/**
	* adds the given force to the given point on the rigid body.
	* both the force and the
	* application point are given in world space. because the
	* force is not applied at the centre of mass, it may be split
	* into both a force and torque.
	*/
	void addforceatpoint(const _vec3 &force, const _vec3 &point);

	/**
	* adds the given force to the given point on the rigid body.
	* the direction of the force is given in world coordinates,
	* but the application point is given in body space. this is
	* useful for spring forces, or other forces fixed to the
	* body.
	*/
	void addforceatbodypoint(const _vec3 &force, const _vec3 &point);

	/**
	* adds the given torque to the rigid body.
	* the force is expressed in world-coordinates.
	*/
	void addtorque(const _vec3 &torque);

	/**
	* sets the constant acceleration of the rigid body
	*/
	void setacceleration(const _vec3 &acceleration);

	/**
	* sets the constant acceleration of the rigid body by component
	*/
	void setacceleration(const float x, const float y, const float z);

	/**
	* fills the given vector with the acceleration of the rigid body
	*/
	void getacceleration(_vec3 *acceleration) const;

	/**
	* gets the acceleration of the rigid body
	*/
	_vec3 getacceleration() const;


};