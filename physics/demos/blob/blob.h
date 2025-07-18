#pragma once

#include "application_header.h"
#include "physics.h"


#define blob_count 5
#define platform_count 10
#define blob_radius 0.4f

/**
* Platforms are two dimensional: lines on which the
* particles can rest. Platforms are also contact generators for the physics.
*/
struct platform : public particle_contact_generator {

	_vec3 m_start;
	_vec3 m_end;

	/**
	* Holds a pointer to the particles we're checking for collisions with.
	*/
	particle *m_particles;

	uint32_t addcontact( particle_contact *contact, uint32_t limit) const {

		const static float restitution = 0.0f;

		uint32_t used = 0;
		for (uint32_t i = 0; i < blob_count; i++)
		{
			if (used >= limit) break;

			// Check for penetration
			_vec3 toparticle = m_particles[i].getposition() - m_start;
			_vec3 linedirection = m_end - m_start;
			float projected = _dot( toparticle , linedirection);
			float platformsqlength = linedirection.squaremagnitude();

			if (projected <= 0) {
				// The blob is nearest to the start point
				if (toparticle.squaremagnitude() < blob_radius*blob_radius) {
					// We have a collision
					contact->m_contact_normal = toparticle.normal();
					contact->m_contact_normal.z = 0;
					contact->m_restitution = restitution;
					contact->m_contact_particles[0] = m_particles + i;
					contact->m_contact_particles[1] = 0;
					contact->m_penetration = blob_radius - toparticle.magnitude();
					used ++;
					contact ++;
				}

			} else if (projected >= platformsqlength) {
				// The blob is nearest to the end point
				toparticle = m_particles[i].getposition() - m_end;
				if (toparticle.squaremagnitude() < blob_radius*blob_radius)
				{
					// We have a collision
					contact->m_contact_normal = toparticle.normal();
					contact->m_contact_normal.z = 0;
					contact->m_restitution = restitution;
					contact->m_contact_particles[0] = m_particles + i;
					contact->m_contact_particles[1] = 0;
					contact->m_penetration = blob_radius - toparticle.magnitude();
					used ++;
					contact ++;
				}
			}
			else
			{
				// the blob is nearest to the middle.
				float distancetoplatform = toparticle.squaremagnitude() - projected*projected / platformsqlength;

				if (distancetoplatform < blob_radius*blob_radius) {
					// We have a collision
					_vec3 closestPoint = m_start + linedirection*(projected/platformsqlength);

					contact->m_contact_normal = (m_particles[i].getposition()-closestPoint).normal();
					contact->m_contact_normal.z = 0;
					contact->m_restitution = restitution;
					contact->m_contact_particles[0] = m_particles + i;
					contact->m_contact_particles[1] = 0;
					contact->m_penetration = blob_radius - sqrt(distancetoplatform);
					used ++;
					contact ++;
				}
			}
		}
		return used;
	}
};

/**
* A force generator for proximal attraction.
*/
struct blob_force_generator : public particle_force_generator {

	/**
	* Holds a pointer to the particles we might be attracting.
	*/
	particle *m_particles;

	/**
	* The maximum force used to push the particles apart.
	*/
	float m_max_replusion;

	/**
	* The maximum force used to pull particles together.
	*/
	float m_max_attraction;

	/**
	* The separation between particles where there is no force.
	*/
	float m_min_natural_distance;

	float m_max_natural_distance;

	/**
	* The force with which to float the head particle, if it is
	* joined to others.
	*/
	float m_float_head;

	/**
	* The maximum number of particles in the blob before the head
	* is floated at maximum force.
	*/
	uint32_t m_max_float;

	/**
	* The separation between particles after which they 'break' apart and
	* there is no force.
	*/
	float m_max_distance;

	void updateforce(particle *_particle, float duration) {

		uint32_t joincount = 0;

		for (uint32_t i = 0; i < blob_count; i++) {
			// Don't attract yourself
			if (m_particles + i == _particle) { continue; }

			// Work out the separation distance
			_vec3 separation = m_particles[i].getposition() - _particle->getposition();
			separation.z = 0.0f;
			float distance = separation.magnitude();

			if (distance < m_min_natural_distance) {
				// Use a repulsion force.
				distance = 1.0f - distance / m_min_natural_distance;
				_particle->addforce(  separation.normal() * (1.0f - distance) * m_max_replusion * -1.0f );
				joincount++;
			}
			else if (distance > m_max_natural_distance && distance < m_max_distance)
			{
				// Use an attraction force.
				distance =
					(distance - m_max_natural_distance) /
					(m_max_distance - m_max_natural_distance);
				_particle->addforce( separation.normal() * distance * m_max_attraction );
				joincount++;
			}
		}

		// If the particle is the head, and we've got a join count, then float it.
		if (_particle == m_particles && joincount > 0 && m_max_float > 0)
		{
			float force = float(joincount / m_max_float) * m_float_head;
			if (force > m_float_head) force = m_float_head;
			_particle->addforce(_vec3(0, force, 0));
		}

	}

};

struct blob : public application_object {

	blob();

	_string        m_state_string;

	particle_world m_world;

	particle *     m_blobs;

	platform *     m_platforms;


	blob_force_generator m_blob_force_generator;

	/* The control for the x-axis. */
	float m_xaxis;

	/* The control for the y-axis. */
	float m_yaxis;

	/**
	* vertex buffer for drawing platforms (lines)
	*/
	IDirect3DVertexBuffer9* m_buffer;

	virtual bool init();
	virtual void clear();
	virtual bool update();

	bool render();

	void reset();

	virtual _string  getstatestring() { return m_state_string; }

	virtual void  key(uint8_t key);
};
