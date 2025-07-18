#include "plinks.h"

float particle_link::currentlength() const {
    _vec3 relativepos = m_contact_particles[0]->getposition() -  m_contact_particles[1]->getposition();
    return relativepos.magnitude();
}

uint32_t particle_cable::addcontact(particle_contact *contact,uint32_t limit) const {

    // find the length of the cable
    float length = currentlength();

    // check if we're over-extended
    if (length < m_max_length) { return 0; }

    // otherwise return the contact
    contact->m_contact_particles[0] = m_contact_particles[0];
    contact->m_contact_particles[1] = m_contact_particles[1];

    // calculate the normal
    _vec3 normal = m_contact_particles[1]->getposition() - m_contact_particles[0]->getposition();
    normal.normalise();
    contact->m_contact_normal = normal;

    contact->m_penetration = length-m_max_length;
    contact->m_restitution = m_restitution;

    return 1;
}

uint32_t particle_rod::addcontact(particle_contact *contact,uint32_t limit) const
{
    // find the length of the rod
    float currentlen = currentlength();

    // check if we're over-extended
    if (currentlen == m_length) { return 0; }

    // otherwise return the contact
    contact->m_contact_particles[0] = m_contact_particles[0];
    contact->m_contact_particles[1] = m_contact_particles[1];

    // calculate the normal
    _vec3 normal = m_contact_particles[1]->getposition() - m_contact_particles[0]->getposition();
    normal.normalise();

    // the contact normal depends on whether we're extending or compressing
    if (currentlen > m_length) {
        contact->m_contact_normal = normal;
        contact->m_penetration   = currentlen - m_length;
    } else {
        contact->m_contact_normal = normal * -1;
        contact->m_penetration = m_length - currentlen;
    }

    // always use zero restitution (no bounciness)
    contact->m_restitution = 0;

    return 1;
}

float particle_constraint::currentlength() const {
    _vec3 relativepos = m_particle->getposition() - m_anchor;
    return relativepos.magnitude();
}

uint32_t particle_cable_constraint::addcontact(particle_contact *contact,uint32_t limit) const {
	
    // find the length of the cable
    float length = currentlength();

    // check if we're over-extended
    if ( length < m_max_length){ return 0; }

    // Otherwise return the contact
    contact->m_contact_particles[0] = m_particle;
    contact->m_contact_particles[1] = 0;

    // calculate the normal
    _vec3 normal = m_anchor - m_particle->getposition();
    normal.normalise();
    contact->m_contact_normal = normal;

    contact->m_penetration = length-m_max_length;
    contact->m_restitution = m_restitution;

    return 1;
}

uint32_t particle_rod_constraint::addcontact(particle_contact *contact,uint32_t limit) const {
    // find the length of the rod
    float currentlen = currentlength();

    // check if we're over-extended
    if (currentlen == m_length) { return 0; }

    // otherwise return the contact
    contact->m_contact_particles[0] = m_particle;
    contact->m_contact_particles[1] = 0;

    // calculate the normal
    _vec3 normal = m_anchor - m_particle->getposition();
    normal.normalise();

    // the contact normal depends on whether we're extending or compressing
    if (currentlen > m_length) {
        contact->m_contact_normal = normal;
        contact->m_penetration = currentlen - m_length;
    } else {
        contact->m_contact_normal = normal * -1;
        contact->m_penetration = m_length - currentlen;
    }

    // always use zero restitution (no bounciness)
    contact->m_restitution = 0;

    return 1;
}