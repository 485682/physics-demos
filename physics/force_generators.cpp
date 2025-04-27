
#include "force_generators.h"

void force_registry::updateforces(float duration) {
    for (uint32_t i = 0; i<m_registrations.m_count; i++) {
		force_registration & registration = m_registrations[i];
		registration.m_fg->updateforce( registration.m_body , duration);
    }
}

void force_registry::add(rigid_body *body, force_generator *fg) {
    force_registration registration;
    registration.m_body = body;
    registration.m_fg = fg;
    m_registrations.pushback(registration,true);
}

buoyancy::buoyancy(const _vec3 &cofb, float maxdepth, float volume,
                   float waterheight, float liquiddensity /* = 1000.0f */) {
    m_centre_of_buoyancy = cofb;
    m_liquid_density     = liquiddensity;
    m_max_depth          = maxdepth;
    m_volume             = volume;
    m_water_height       = waterheight;
}

void buoyancy::updateforce(rigid_body *body, float duration) {
    // calculate the submersion depth
    _vec3 pointinworld = body->getpointinworldspace(m_centre_of_buoyancy);
    float depth = pointinworld.y;

    // check if we're out of the water
	if (depth >= m_water_height + m_max_depth) { return; }
    _vec3 force(0,0,0);

    // check if we're at maximum depth
    if (depth <= m_water_height - m_max_depth) {
        force.y = m_liquid_density * m_volume;
        body->addforceatbodypoint(force, m_centre_of_buoyancy );
        return;
    }

    // otherwise we are partly submerged
    force.y = m_liquid_density * m_volume *
        (depth - m_max_depth - m_water_height) / 2 * m_max_depth;
    body->addforceatbodypoint(force, m_centre_of_buoyancy);
}

gravity::gravity(const _vec3& gravity) : m_gravity(gravity) { }

void gravity::updateforce(rigid_body* body, float duration) {
    // check that we do not have infinite mass
	if (!body->hasfinitemass()) { return; }

    // apply the mass-scaled force to the body
    body->addforce(m_gravity * body->getmass());
}

spring::spring(const _vec3 &localconnectionpt,
               rigid_body *other,
               const _vec3 &otherconnectionpt,
               float springconstant,
               float restlength)
: m_connection_point(localconnectionpt),
  m_other_connection_point(otherconnectionpt),
  m_other(other),
  m_spring_constant(springconstant),
  m_rest_length(restlength)
{
}

void spring::updateforce(rigid_body* body, float duration) {
    // calculate the two ends in world space
    _vec3 lws = body->getpointinworldspace(m_connection_point);
    _vec3 ows = m_other->getpointinworldspace(m_other_connection_point);

    // calculate the vector of the spring
    _vec3 force = lws - ows;

    // calculate the magnitude of the force
    float magnitude = force.magnitude();
    magnitude = abs(magnitude - m_rest_length);
    magnitude *= m_spring_constant;

    // calculate the final force and apply it
    force.normalise();
    force *= -magnitude;
    body->addforceatpoint(force, lws);
}

aero::aero(const _mat3 &tensor, const _vec3 &position, const _vec3 *windspeed) {
    m_tensor    = tensor;
    m_position  = position;
    m_wind_speed = windspeed;
}

void aero::updateforce(rigid_body *body, float duration) {
    updateforcefromtensor(body, duration, m_tensor);
}

void aero::updateforcefromtensor(rigid_body *body, float duration,
                                 const _mat3 &tensor) {
    // calculate total velocity (windspeed and body's velocity).
    _vec3 velocity = body->getvelocity();
    velocity += * m_wind_speed;

    // calculate the velocity in body coordinates
    _vec3 bodyvel = body->gettransform().transforminversedirection(velocity);

    // calculate the force in body coordinates
    _vec3 bodyforce = tensor * bodyvel;
    _vec3 force = body->gettransform().transformdirection(bodyforce);

    // apply the force
    body->addforceatbodypoint(force, m_position);
}

aero_control::aero_control(const _mat3 &base, const _mat3 &min, const _mat3 &max,
                              const _vec3 &position, const _vec3 *windspeed)
: aero(base, position, windspeed) {
    m_min_tensor = min;
    m_max_tensor = max;
    m_control_setting = 0.0f;
}

_mat3 aero_control::gettensor() {

    if (m_control_setting <= -1.0f) return m_min_tensor;
    else if (m_control_setting >= 1.0f) return m_max_tensor;
    else if (m_control_setting < 0){
        return _mat3::linearinterpolate( m_min_tensor, m_tensor, m_control_setting+1.0f);
    }else if (m_control_setting > 0) {
        return _mat3::linearinterpolate(m_tensor, m_max_tensor, m_control_setting);
    }
    else return m_tensor;
}

void aero_control::setcontrol(float value) { m_control_setting = value; }

void aero_control::updateforce(rigid_body *body, float duration) {
    _mat3 tensor = gettensor();
    aero::updateforcefromtensor(body, duration, tensor);
}
