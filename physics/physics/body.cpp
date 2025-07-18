#include "body.h"

#include <memory.h>


/**
* Internal function to do an intertia tensor transform
*/
static inline void _transforminertiatensor(
	_mat3 &iitworld,
	const _mat3 &iitbody,
	const _mat4 &rotmat) {

		float t4  = rotmat[0][0]*iitbody[0][0] + rotmat[1][0]*iitbody[0][1] + rotmat[2][0]*iitbody[0][2];
		float t9  = rotmat[0][0]*iitbody[1][0] + rotmat[1][0]*iitbody[1][1] + rotmat[2][0]*iitbody[1][2];
		float t14 = rotmat[0][0]*iitbody[2][0] + rotmat[1][0]*iitbody[2][1] + rotmat[2][0]*iitbody[2][2];
		float t28 = rotmat[0][1]*iitbody[0][0] + rotmat[1][1]*iitbody[0][1] + rotmat[2][1]*iitbody[0][2]; 
		float t33 = rotmat[0][1]*iitbody[1][0] + rotmat[1][1]*iitbody[1][1] + rotmat[2][1]*iitbody[1][2];
		float t38 = rotmat[0][1]*iitbody[2][0] + rotmat[1][1]*iitbody[2][1] + rotmat[2][1]*iitbody[2][2];
		float t52 = rotmat[0][2]*iitbody[0][0] + rotmat[1][2]*iitbody[0][1] + rotmat[2][2]*iitbody[0][2];
		float t57 = rotmat[2][2]*iitbody[1][0] + rotmat[1][2]*iitbody[1][1] + rotmat[2][2]*iitbody[1][2];
		float t62 = rotmat[0][2]*iitbody[2][0] + rotmat[1][2]*iitbody[2][1] + rotmat[2][2]*iitbody[2][2];

		iitworld[0][0] = t4*rotmat[0][0]  + t9*rotmat[1][0]  + t14*rotmat[2][0];
		iitworld[1][0] = t4*rotmat[0][1]  + t9*rotmat[1][1]  + t14*rotmat[2][1];
		iitworld[2][0] = t4*rotmat[0][2]  + t9*rotmat[1][2]  + t14*rotmat[2][2];
		iitworld[0][1] = t28*rotmat[0][0] + t33*rotmat[1][0] + t38*rotmat[2][0];
		iitworld[1][1] = t28*rotmat[0][1] + t33*rotmat[1][1] + t38*rotmat[2][1];
		iitworld[2][1] = t28*rotmat[0][2] + t33*rotmat[1][2] + t38*rotmat[2][2];
		iitworld[0][2] = t52*rotmat[0][0] + t57*rotmat[1][0] + t62*rotmat[2][0];
		iitworld[1][2] = t52*rotmat[0][1] + t57*rotmat[1][1] + t62*rotmat[2][1];
		iitworld[2][2] = t52*rotmat[0][2] + t57*rotmat[1][2] + t62*rotmat[2][2];
}

/**
* Inline function that creates a transform matrix from a position
*/
static inline void _calculatetransformmatrix(_mat4 &transformmatrix,
	const _vec3 &position,
	const _quaternion &orientation) {
		transformmatrix[0][0] = 1-2*orientation.j*orientation.j  - 2*orientation.k*orientation.k;
		transformmatrix[1][0] = 2*orientation.i*orientation.j    - 2*orientation.r*orientation.k;
		transformmatrix[2][0] = 2*orientation.i*orientation.k    + 2*orientation.r*orientation.j;
		transformmatrix[3][0] = position.x;

		transformmatrix[0][1] = 2*orientation.i*orientation.j    + 2*orientation.r*orientation.k;
		transformmatrix[1][1] = 1-2*orientation.i*orientation.i  - 2*orientation.k*orientation.k;
		transformmatrix[2][1] = 2*orientation.j*orientation.k    - 2*orientation.r*orientation.i;
		transformmatrix[3][1] = position.y;

		transformmatrix[0][2] = 2*orientation.i*orientation.k    - 2*orientation.r*orientation.j;
		transformmatrix[1][2] = 2*orientation.j*orientation.k    + 2*orientation.r*orientation.i;
		transformmatrix[2][2] = 1-2*orientation.i*orientation.i - 2*orientation.j*orientation.j;
		transformmatrix[3][2] = position.z;
}

void rigid_body::calculatederiveddata() {

	m_orientation.normalise();

	// Calculate the transform matrix for the body.
	_calculatetransformmatrix(m_transform_matrix, m_position, m_orientation);

	// Calculate the inertiaTensor in world space.
	_transforminertiatensor(
		m_inverse_inertia_tensor_world,
		m_inverse_inertia_tensor,
		m_transform_matrix);

}

void rigid_body::integrate(float duration) {

	if (!m_isawake) { return; }

	// calculate linear acceleration from force inputs.
	m_last_frame_acceleration = m_acceleration;
	m_last_frame_acceleration.addscaledvector( m_force_accumulated, m_inverse_mass );

	// calculate angular acceleration from torque inputs.
	_vec3 angularacceleration = m_inverse_inertia_tensor_world * m_torque_accumulated ;

	// adjust velocities
	// update linear velocity from both acceleration and impulse.
	m_velocity.addscaledvector(m_last_frame_acceleration, duration);

	// update angular velocity from both acceleration and impulse.
	m_rotation.addscaledvector(angularacceleration, duration);

	// impose drag.
	m_velocity *= pow( m_linear_damping , duration);
	m_rotation *= pow( m_angular_damping, duration);

	// adjust positions
	// update linear position.
	m_position.addscaledvector(m_velocity, duration);

	// update angular position.
	m_orientation.addscaledvector(m_rotation, duration);

	// normalise the orientation, and update the matrices with the new
	// position and orientation
	calculatederiveddata();

	// clear accumulators.
	clearaccumulators();

	// update the kinetic energy store, and possibly put the body to
	// sleep.
	if (m_cansleep) {
		float currentmotion = _dot(m_velocity,m_velocity) + _dot(m_rotation,m_rotation);

		float bias = pow(0.5f, duration);
		m_motion = bias*m_motion + (1-bias)*currentmotion;

		if ( m_motion<_sleepepsilon ) { setawake(false); }
		else if (m_motion > 10 * _sleepepsilon) { m_motion = 10 * _sleepepsilon; }
	}
}

void rigid_body::setmass(const float mass) {
	if(mass != 0) { m_inverse_mass = 1.0f/mass; }
}

float rigid_body::getmass() const {
	if (m_inverse_mass == 0) {
		return FLT_MAX;
	} else {
		return 1.0f/m_inverse_mass;
	}
}

void rigid_body::setinversemass(const float inversemass) { m_inverse_mass = inversemass; }

float rigid_body::getinversemass() const { return m_inverse_mass; }

bool rigid_body::hasfinitemass() const { return m_inverse_mass >= 0.0f; }

void rigid_body::setinertiatensor(const _mat3 &inertiatensor) {
	m_inverse_inertia_tensor.setinverse(inertiatensor);
}

void rigid_body::getinertiatensor(_mat3 *inertiatensor) const {
	inertiatensor->setinverse(m_inverse_inertia_tensor);
}

_mat3 rigid_body::getinertiatensor() const {
	_mat3 it;
	getinertiatensor(&it);
	return it;
}

void rigid_body::getinertiatensorworld(_mat3 *inertiatensor) const {
	inertiatensor->setinverse( m_inverse_inertia_tensor_world );
}

_mat3 rigid_body::getinertiatensorworld() const {
	_mat3 it;
	getinertiatensorworld(&it);
	return it;
}

void rigid_body::setinverseinertiatensor(const _mat3 &inverseinertiatensor) {
	m_inverse_inertia_tensor = inverseinertiatensor;
}

void rigid_body::getinverseinertiatensor(_mat3 *inverseinertiatensor) const {
	*inverseinertiatensor = m_inverse_inertia_tensor;
}

_mat3 rigid_body::getinverseinertiatensor() const { return m_inverse_inertia_tensor; }

void rigid_body::getinverseinertiatensorworld(_mat3 *inverseinertiatensor) const {
	*inverseinertiatensor = m_inverse_inertia_tensor_world;
}

_mat3 rigid_body::getinverseinertiatensorworld() const { return m_inverse_inertia_tensor_world; }

void rigid_body::setdamping(const float lineardamping, const float angulardamping) {
	m_linear_damping = lineardamping;
	m_angular_damping = angulardamping;
}

void rigid_body::setlineardamping(const float lineardamping) { m_linear_damping = lineardamping; }

float rigid_body::getlineardamping() const { return m_linear_damping; }

void rigid_body::setangulardamping(const float angulardamping) { m_angular_damping = angulardamping; }

float rigid_body::getangulardamping() const { return m_angular_damping; }

void rigid_body::setposition(const _vec3 &position) { m_position = position; }

void rigid_body::setposition(const float x, const float y, const float z) {
	m_position.x = x;
	m_position.y = y;
	m_position.z = z;
}

void rigid_body::getposition(_vec3 *position) const { *position = m_position; }

_vec3 rigid_body::getposition() const { return m_position; }

void rigid_body::setorientation(const _quaternion &orientation) {
	m_orientation = orientation;
	m_orientation.normalise();
}

void rigid_body::setorientation(const float r, const float i, const float j, const float k) {
	m_orientation.r = r;
	m_orientation.i = i;
	m_orientation.j = j;
	m_orientation.k = k;
	m_orientation.normalise();
}

void rigid_body::getorientation(_quaternion *orientation) const { *orientation = m_orientation; }

_quaternion rigid_body::getorientation() const { return m_orientation; }

void rigid_body::getorientation(_mat3 *matrix) const {
	getorientation( (float*)matrix->m_data );
}

void rigid_body::getorientation(float matrix[9]) const {

	matrix[0] = m_transform_matrix[0][0];
	matrix[1] = m_transform_matrix[1][0];
	matrix[2] = m_transform_matrix[2][0];

	matrix[3] = m_transform_matrix[0][1];
	matrix[4] = m_transform_matrix[1][1];
	matrix[5] = m_transform_matrix[2][1];

	matrix[6] = m_transform_matrix[0][2];
	matrix[7] = m_transform_matrix[1][2];
	matrix[8] = m_transform_matrix[2][2];
}

void rigid_body::gettransform(_mat4 *transform) const {
	memcpy(transform, &m_transform_matrix.m_data, sizeof(_mat4)); }

void rigid_body::gettransform(float matrix[16]) const
{
	memcpy(matrix, m_transform_matrix.m_data, sizeof(float)*12);
	matrix[12] = matrix[13] = matrix[14] = 0;
	matrix[15] = 1;
}

_mat4 rigid_body::gettransform() const { return m_transform_matrix; }

_vec3 rigid_body::getpointinlocalspace(const _vec3 &point) const {
	return m_transform_matrix.transforminverse(point);
}

_vec3 rigid_body::getpointinworldspace(const _vec3 &point) const {
	return m_transform_matrix* point;
}

_vec3 rigid_body::getdirectioninlocalspace(const _vec3 &direction) const {
	return m_transform_matrix.transforminversedirection(direction);
}

_vec3 rigid_body::getdirectioninworldspace(const _vec3 &direction) const {
	return m_transform_matrix.transformdirection(direction);
}

void rigid_body::setvelocity(const _vec3 &velocity) { m_velocity = velocity; }

void rigid_body::setvelocity(const float x, const float y, const float z) {
	m_velocity.x = x;
	m_velocity.y = y;
	m_velocity.z = z;
}

void rigid_body::getvelocity(_vec3 *velocity) const { *velocity = m_velocity; }

_vec3 rigid_body::getvelocity() const { return m_velocity; }

void rigid_body::addvelocity(const _vec3 &deltavelocity) { m_velocity += deltavelocity; }

void rigid_body::setrotation(const _vec3 &rotation) { m_rotation = rotation; }

void rigid_body::setrotation(const float x, const float y, const float z) {
	m_rotation.x = x;
	m_rotation.y = y;
	m_rotation.z = z;
}

void rigid_body::getrotation(_vec3 *rotation) const { *rotation = m_rotation; }

_vec3 rigid_body::getrotation() const { return m_rotation; }

void rigid_body::addrotation(const _vec3 &deltarotation) { m_rotation += deltarotation; }

void rigid_body::setawake(const bool awake) {
	if (awake) {
		m_isawake= true;

		// add a bit of motion to avoid it falling asleep immediately.
		m_motion = _sleepepsilon*2.0f;
	} else {
		m_isawake = false;
		m_velocity.clear();
		m_rotation.clear();
	}
}

void rigid_body::setcansleep(const bool cansleep) {
	m_cansleep = cansleep;
	if (!cansleep && !m_isawake) { setawake(); }
}

void rigid_body::getlastframeacceleration(_vec3 *acceleration) const { *acceleration = m_last_frame_acceleration; }

_vec3 rigid_body::getlastframeacceleration() const { return m_last_frame_acceleration; }

void rigid_body::clearaccumulators() {
	m_force_accumulated.clear();
	m_torque_accumulated.clear();
}

void rigid_body::addforce(const _vec3 &force) {
	m_force_accumulated += force;
	m_isawake = true;
}

void rigid_body::addforceatbodypoint(const _vec3 &force, const _vec3 &point) {
	// convert to coordinates relative to center of mass.
	_vec3 pt = getpointinworldspace(point);
	addforceatpoint(force, pt);

}

void rigid_body::addforceatpoint(const _vec3 &force, const _vec3 &point) {
	// convert to coordinates relative to center of mass.
	_vec3 pt = point;
	pt -= m_position;

	m_force_accumulated += force;
	m_torque_accumulated += _cross(pt,force);

	m_isawake = true;
}

void rigid_body::addtorque(const _vec3 &torque) {
	m_torque_accumulated += torque;
	m_isawake = true;
}

void rigid_body::setacceleration(const _vec3 &acceleration) { m_acceleration = acceleration; }

void rigid_body::setacceleration(const float x, const float y, const float z) {
	m_acceleration.x = x;
	m_acceleration.y = y;
	m_acceleration.z = z;
}

void rigid_body::getacceleration(_vec3 *acceleration) const { *acceleration = m_acceleration; }

_vec3 rigid_body::getacceleration() const { return m_acceleration; }
