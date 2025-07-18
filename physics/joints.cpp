
#include "physics.h"


uint32_t joint::addcontact(contact *contact, uint32_t limit) const {

    // calculate the position of each connection point in world coordinates
    _vec3 a_pos_world = m_body[0]->getpointinworldspace(m_position[0]);
    _vec3 b_pos_world = m_body[1]->getpointinworldspace(m_position[1]);

    // calculate the length of the joint
    _vec3 a_to_b = b_pos_world - a_pos_world;
    _vec3 normal = a_to_b;
    normal.normalise();
    float length = a_to_b.magnitude();

    // check if it is violated
    if ( abs(length) > m_error) {

        contact->m_body[0] = m_body[0];
        contact->m_body[1] = m_body[1];
        contact->m_contact_normal = normal;
        contact->m_contact_point = (a_pos_world + b_pos_world) * 0.5f;
        contact->m_penetration = length-m_error;
        contact->m_friction = 1.0f;
        contact->m_restitution = 0;
        return 1;
    }

    return 0;
}

void joint::set(
	rigid_body *a, const _vec3& a_pos,
	rigid_body *b, const _vec3& b_pos,
	float error
	)
{
    m_body[0] = a;
    m_body[1] = b;

    m_position[0] = a_pos;
    m_position[1] = b_pos;

    m_error = error;
}