#include "pcontacts.h"


void particle_contact::resolve(float duration) { resolvevelocity(duration); resolveinterpenetration(duration); }

float particle_contact::calculateseparatingvelocity() const {
	_vec3 relativevelocity = m_contact_particles[0]->getvelocity();
	if (m_contact_particles[1]) { relativevelocity -= m_contact_particles[1]->getvelocity(); }
	return _dot(relativevelocity,m_contact_normal);
}

void particle_contact::resolvevelocity( float duration) {

	// find the velocity in the direction of the contact
	float separatingvelocity = calculateseparatingvelocity();

	// check if it needs to be resolved
	if (separatingvelocity > 0) {
		// the contact is either separating, or stationary - there's
		// no impulse required.
		return;
	}

	// calculate the new separating velocity
	float newsepvelocity = -separatingvelocity * m_restitution;

	// check the velocity build-up due to acceleration only
	_vec3 acc_causedvelocity = m_contact_particles[0]->getacceleration();
	if (m_contact_particles[1]) { acc_causedvelocity -= m_contact_particles[1]->getacceleration(); }
	float acc_causedsepvelocity = _dot(acc_causedvelocity , m_contact_normal) * duration;

	// if we've got a closing velocity due to acceleration build-up,
	// remove it from the new separating velocity
	if ( acc_causedsepvelocity < 0) {
		newsepvelocity += m_restitution * acc_causedsepvelocity;

		// make sure we haven't removed more than was
		// there to remove.
		if (newsepvelocity < 0) { newsepvelocity = 0; }
	}

	float deltavelocity = newsepvelocity - separatingvelocity;

	// we apply the change in velocity to each object in proportion to
	// their inverse mass (i.e. those with lower inverse mass [higher
	// actual mass] get less change in velocity)..
	float totalinversemass = m_contact_particles[0]->getinversemass();
	if (m_contact_particles[1]) { totalinversemass += m_contact_particles[1]->getinversemass(); }

	// if all particles have infinite mass, then impulses have no effect
	if (totalinversemass <= 0) { return; }

	// calculate the impulse to apply
	float impulse = deltavelocity / totalinversemass;

	// find the amount of impulse per unit of inverse mass
	_vec3 impulseperimass = m_contact_normal * impulse;

	// apply impulses: they are applied in the direction of the contact,
	// and are proportional to the inverse mass.
	m_contact_particles[0]->setvelocity(m_contact_particles[0]->getvelocity() + impulseperimass * m_contact_particles[0]->getinversemass() );
	if (m_contact_particles[1]){
		// particle 1 goes in the opposite direction
		m_contact_particles[1]->setvelocity(m_contact_particles[1]->getvelocity() + impulseperimass * -m_contact_particles[1]->getinversemass() );
	}
}

void particle_contact::resolveinterpenetration(float duration) {

	// if we don't have any penetration, skip this step.
	if (m_penetration <= 0) { return; } 

	// the movement of each object is based on their inverse mass, so
	// total that.
	float totalinversemass = m_contact_particles[0]->getinversemass();
	if (m_contact_particles[1]) { totalinversemass += m_contact_particles[1]->getinversemass(); }

	// if all particles have infinite mass, then we do nothing
	if (totalinversemass <= 0) { return; }

	// find the amount of penetration resolution per unit of inverse mass
	_vec3 moveperimass = m_contact_normal * (m_penetration / totalinversemass);

	// calculate the the movement amounts
	m_particle_movement[0] = moveperimass * m_contact_particles[0]->getinversemass();
	if (m_contact_particles[1]) {
		m_particle_movement[1] = moveperimass * -m_contact_particles[1]->getinversemass();
	} else {
		m_particle_movement[1].clear();
	}

	// apply the penetration resolution
	m_contact_particles[0]->setposition(m_contact_particles[0]->getposition() + m_particle_movement[0]);
	if (m_contact_particles[1]) {
		m_contact_particles[1]->setposition(m_contact_particles[1]->getposition() + m_particle_movement[1]);
	}
}

particle_contact_resolver::particle_contact_resolver(uint32_t iterations) : m_iterations(iterations) { }

void particle_contact_resolver::setiterations(uint32_t iterations) {
	particle_contact_resolver::m_iterations = iterations;
}

void particle_contact_resolver::resolvecontacts(particle_contact *contactarray,uint32_t numcontacts,float duration){

	uint32_t i;

	m_iterations_used = 0;
	while(m_iterations_used < m_iterations) {
		// find the contact with the largest closing velocity;
		float max = FLT_MAX;
		uint32_t maxindex = numcontacts;
		for (i = 0; i < numcontacts; i++){
			float sepvel = contactarray[i].calculateseparatingvelocity();
			if (sepvel < max && (sepvel < 0 || contactarray[i].m_penetration > 0) ) {
				max = sepvel;
				maxindex = i;
			}
		}

		// do we have anything worth resolving?
		if (maxindex == numcontacts) { break; }

		// resolve this contact
		contactarray[maxindex].resolve(duration);

		// update the interpenetrations for all particles
		_vec3 * move = contactarray[maxindex].m_particle_movement;
		for (i = 0; i < numcontacts; i++) {
			if (contactarray[i].m_contact_particles[0] == contactarray[maxindex].m_contact_particles[0]) {
				contactarray[i].m_penetration -= _dot(move[0] ,contactarray[i].m_contact_normal) ;
			} else if (contactarray[i].m_contact_particles[0] == contactarray[maxindex].m_contact_particles[1]) {
				contactarray[i].m_penetration -= _dot(move[1] , contactarray[i].m_contact_normal) ;
			}
			if (contactarray[i].m_contact_particles[1]) {
				if (contactarray[i].m_contact_particles[1] == contactarray[maxindex].m_contact_particles[0]){
					contactarray[i].m_penetration += _dot(move[0] ,contactarray[i].m_contact_normal);
				} else if (contactarray[i].m_contact_particles[1] == contactarray[maxindex].m_contact_particles[1]) {
					contactarray[i].m_penetration += _dot(move[1] , contactarray[i].m_contact_normal);
				}
			}
		}
		m_iterations_used++;
	}
}
