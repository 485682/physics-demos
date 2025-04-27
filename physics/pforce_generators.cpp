#include "pforce_generators.h"


particle_gravity::particle_gravity(const _vec3& gravity) : m_gravity(gravity) { }

void particle_gravity::updateforce(particle* _particle, float duration) {
    // check that we do not have infinite mass
    if (!_particle->hasfinitemass()) { return; }

    // apply the mass-scaled force to the particle
    _particle->addforce(m_gravity * _particle->getmass());
}

particle_drag::particle_drag(float k1, float k2) : m_k1(k1), m_k2(k2) { }

void particle_drag::updateforce(particle* _particle, float duration){
    _vec3 force;
    _particle->getvelocity(&force);

    // calculate the total drag coefficient
    float dragCoeff = force.magnitude();
    dragCoeff = m_k1 * dragCoeff + m_k2 * dragCoeff * dragCoeff;

    // calculate the final force and apply it
    force.normalise();
    force *= -dragCoeff;
    _particle->addforce(force);
}

particle_spring::particle_spring(particle *other, float sc, float rl) : m_other(other), m_spring_constant(sc), m_rest_length(rl) { }

void particle_spring::updateforce(particle* _particle, float duration) {
	
    // calculate the vector of the spring
    _vec3 force;
    _particle->getposition(&force);
    force -= m_other->getposition();

    // calculate the magnitude of the force
    float magnitude = force.magnitude();
    magnitude = abs(magnitude - m_rest_length);
    magnitude *= m_spring_constant;

    // calculate the final force and apply it
    force.normalise();
    force *= -magnitude;
    _particle->addforce(force);
}


particle_anchored_spring::particle_anchored_spring() { }

particle_anchored_spring::particle_anchored_spring( _vec3 *anchor,float sc, float rl)
: m_anchor(anchor), m_spring_constant(sc), m_rest_length(rl) { }

void particle_anchored_spring::init( _vec3 *anchor, float springconstant, float restlength){
    particle_anchored_spring::m_anchor         = anchor;
    particle_anchored_spring::m_spring_constant = springconstant;
    particle_anchored_spring::m_rest_length     = restlength;
}


void particle_anchored_spring::updateforce( particle* _particle, float duration) {
    // calculate the vector of the spring
    _vec3 force;
    _particle->getposition(&force);
    force -= *m_anchor;

    // calculate the magnitude of the force
    float magnitude = force.magnitude();
    magnitude = (m_rest_length - magnitude) * m_spring_constant;

    // calculate the final force and apply it
    force.normalise();
    force *= magnitude;
    _particle->addforce(force);
}

particle_fake_spring::particle_fake_spring(_vec3 *anchor, float sc, float d)
: m_anchor(anchor), m_spring_constant(sc), m_damping(d) { }

void particle_fake_spring::updateforce(particle* _particle, float duration) {
    // check that we do not have infinite mass
    if (!_particle->hasfinitemass()) { return; }

    // calculate the relative position of the particle to the anchor
    _vec3 position;
    _particle->getposition(&position);
    position -= *m_anchor;

    // calculate the constants and check they are in bounds.
    float gamma = 0.5f * sqrt(4 * m_spring_constant - m_damping*m_damping);
	if (gamma == 0.0f) { return; }
    _vec3 c = position * (m_damping / (2.0f * gamma)) + _particle->getvelocity() * (1.0f / gamma);

    // calculate the target position
    _vec3 target = position * cos(gamma * duration) + c * sin(gamma * duration);
	
    target *= exp(-0.5f * duration * m_damping);

    // calculate the resulting acceleration and therefore the force
    _vec3 accel = (target - position) * ((float)1.0 / (duration*duration)) - _particle->getvelocity() * ((float)1.0/duration);
    _particle->addforce(accel * _particle->getmass());
}

void particle_anchored_bungee::updateforce( particle* _particle, float duration) {
    // calculate the vector of the spring
    _vec3 force;
    _particle->getposition(&force);
    force -= *m_anchor;

    // calculate the magnitude of the force
    float magnitude = force.magnitude();
    if (magnitude < m_rest_length) { return; }

    magnitude = magnitude - m_rest_length;
    magnitude *= m_spring_constant;

    // calculate the final force and apply it
    force.normalise();
    force *= -magnitude;
    _particle->addforce(force);
}

particle_bungee::particle_bungee(particle *other, float sc, float rl)
: m_other(other), m_spring_constant(sc), m_rest_length(rl) { }

void particle_bungee::updateforce(particle* _particle, float duration) {
    // calculate the vector of the spring
    _vec3 force;
    _particle->getposition(&force);
    force -= m_other->getposition();

    // check if the bungee is compressed
    float magnitude = force.magnitude();
	if (magnitude <= m_rest_length) { return; } 

    // calculate the magnitude of the force
    magnitude = m_spring_constant * ( m_rest_length - magnitude);

    // calculate the final force and apply it
    force.normalise();
    force *= -magnitude;
    _particle->addforce(force);
}

particle_buoyancy::particle_buoyancy(float maxdepth, float volume, float waterheight, float liquiddensity)
: m_max_depth(maxdepth), m_volume(volume), m_water_height(waterheight), m_liquid_density(liquiddensity) { }

void particle_buoyancy::updateforce(particle* _particle, float duration){
    // calculate the submersion depth
    float depth = _particle->getposition().y;

    // check if we're out of the water
    if (depth >= m_water_height + m_max_depth) { return; }
	
    _vec3 force(0,0,0);

    // check if we're at maximum depth
    if (depth <= m_water_height - m_max_depth){
        force.y = m_liquid_density * m_volume;
        _particle->addforce(force);
        return;
    }

    // otherwise we are partly submerged
    force.y = m_liquid_density * m_volume * (depth - m_max_depth - m_water_height) / (2 * m_max_depth);
	
    _particle->addforce(force);
}

void particle_force_registry::updateforces(float duration){

	for (uint32_t i=0; i< m_registrations.m_count; i++){
		m_registrations[i].m_fg->updateforce( m_registrations[i].m_particle, duration);
    }
}

void particle_force_registry::add(particle* _particle, particle_force_generator *fg) {
    particle_force_registry::particle_force_registration registration;
    registration.m_particle = _particle;
    registration.m_fg = fg;
    m_registrations.pushback(registration,true);
}
