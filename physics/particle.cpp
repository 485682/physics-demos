#include "particle.h"


void particle::integrate(float duration){

    // we don't integrate things with zero mass.

	if (m_inverse_mass > 0.0f) { 

		// update linear position.
		m_position.addscaledvector(m_velocity, duration);

		// work out the acceleration from the force
		_vec3 resultingAcc = m_acceleration;
		resultingAcc.addscaledvector(m_forces, m_inverse_mass);

		// update linear velocity from the acceleration.
		m_velocity.addscaledvector(resultingAcc, duration);

		// impose drag.
		m_velocity *=  pow(m_damping, duration);

		// clear the forces.
		clearaccumulator();
	}

}



void particle::setmass(const float mass) {
	if(mass == 0){
		application_error("mass==0"); 
	}else{
		particle::m_inverse_mass = ((float)1.0)/mass;
	}
}

float particle::getmass() const {
    if (m_inverse_mass == 0) {
        return FLT_MAX;
    } else {
        return 1.0f/m_inverse_mass;
    }
}

void  particle::setinversemass(const float inverse_mass) { particle::m_inverse_mass = inverse_mass; }

float particle::getinversemass() const { return m_inverse_mass; }

bool  particle::hasfinitemass()  const { return m_inverse_mass >= 0.0f; }

void  particle::setdamping(const float damping) { particle::m_damping = damping; }

float particle::getdamping() const { return m_damping; }

void  particle::setposition(const _vec3 &position) { particle::m_position = position; }

void particle::setposition(const float x, const float y, const float z) {
    m_position.x = x;
    m_position.y = y;
    m_position.z = z;
}

void particle::getposition(_vec3 *position) const { *position = particle::m_position; }

_vec3 particle::getposition() const { return m_position; }

void particle::setvelocity(const _vec3 &velocity) { particle::m_velocity = velocity; }

void particle::setvelocity(const float x, const float y, const float z){
    m_velocity.x = x;
    m_velocity.y = y;
    m_velocity.z = z;
}

void particle::getvelocity(_vec3 *velocity) const { *velocity = particle::m_velocity; }

_vec3 particle::getvelocity() const { return m_velocity; }

void particle::setacceleration(const _vec3 &acceleration) { particle::m_acceleration = acceleration; }

void particle::setacceleration(const float x, const float y, const float z){
    m_acceleration.x = x;
    m_acceleration.y = y;
    m_acceleration.z = z;
}

void particle::getacceleration(_vec3 *acceleration) const { *acceleration = particle::m_acceleration; }

_vec3 particle::getacceleration() const { return m_acceleration; }

void particle::clearaccumulator() { m_forces.clear(); }

void particle::addforce(const _vec3 &force) { m_forces += force; }
