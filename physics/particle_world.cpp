#include "particle_world.h"

particle_world::particle_world(uint32_t maxcontacts, uint32_t iterations) 
: m_resolver(iterations),m_max_contacts(maxcontacts)
{
    m_contacts = new particle_contact[maxcontacts];
    m_calculate_iterations = (iterations == 0);
}

particle_world::~particle_world() { delete[] m_contacts; }

void particle_world::startframe() {
	for (uint32_t i=0; i<m_particles.m_count; i++) {
        // remove all forces from the accumulator
        m_particles[i]->clearaccumulator();
    }
}

uint32_t particle_world::generatecontacts() {
    uint32_t limit = m_max_contacts;
    particle_contact *nextcontact = m_contacts;

	for (uint32_t i=0; i<m_contact_generators.m_count; i++) {
        uint32_t used = m_contact_generators[i]->addcontact(nextcontact, limit);
        limit -= used;
        nextcontact += used;

        // we've run out of contacts to fill. this means we're missing
        // contacts.
	    if (limit <= 0) { break; }
    }

    // return the number of contacts used.
    return m_max_contacts - limit;
}

void particle_world::integrate(float duration) {
	for (uint32_t i=0; i<m_particles.m_count; i++) {
        // remove all forces from the accumulator
        m_particles[i]->integrate(duration);
    }
}

void particle_world::runphysics(float duration) {
    // first apply the force generators
    m_registry.updateforces(duration);

    // then integrate the objects
    integrate(duration);

    // generate contacts
    uint32_t usedcontacts = generatecontacts();

    // and process them
    if (usedcontacts) {
        if (m_calculate_iterations) { m_resolver.setiterations(usedcontacts * 2); }
        m_resolver.resolvecontacts(m_contacts, usedcontacts, duration);
    }
}


particle_world::_contact_generators& particle_world::getcontactgenerators() { return m_contact_generators; }

particle_force_registry& particle_world::getforceregistry() { return m_registry; }

void ground_contacts::init(particle_world::_particles *particles) { ground_contacts::m_particles = particles; }

uint32_t ground_contacts::addcontact( particle_contact *contact,uint32_t limit) const {
    uint32_t count = 0;
    for ( uint32_t i=0;i<m_particles->m_count;i++)    {
        float y = (*m_particles)[i]->getposition().y;
        if (y < 0.0f) {
            contact->m_contact_normal = _utility::up;
            contact->m_contact_particles[0] = (*m_particles)[i];
            contact->m_contact_particles[1] = NULL;
            contact->m_penetration = -y;
            contact->m_restitution = 0.2f;
            contact++;
            count++;
        }

        if (count >= limit) { return count; }
    }
    return count;
}
