#pragma once

#include "application_header.h"
#include "physics.h"

#define MAX_BLOCKS 9

//random global_random;

struct block : public collision_box {

	
	bool m_exists;

    block() : m_exists(false) { m_body = new rigid_body(); }

    ~block() { delete m_body; }

    /** Sets the block to a specific location. */
    void setstate(const _vec3 &position,
                  const _quaternion &orientation,
                  const _vec3 &extents,
                  const _vec3 &velocity)
    {
        m_body->setposition(position);
        m_body->setorientation(orientation);
        m_body->setvelocity(velocity);
        m_body->setrotation(_vec3(0,0,0));
        m_half_size = extents;

        float mass = m_half_size.x * m_half_size.y * m_half_size.z * 8.0f;
        m_body->setmass(mass);

        _mat3 tensor;
        tensor.setblockinertiatensor(m_half_size, mass);
        m_body->setinertiatensor(tensor);

        m_body->setlineardamping(0.95f);
        m_body->setangulardamping(0.8f);
        m_body->clearaccumulators();
        m_body->setacceleration(0,-10.0f,0);

        //body->setcansleep(false);
        m_body->setawake();

        m_body->calculatederiveddata();
    }

    /**
     * calculates and sets the mass and inertia tensor of this block,
     * assuming it has the given constant density.
     */
    void calculatemassproperties( float invdensity) {
        // check for infinite mass
        if (invdensity <= 0) {
            // just set zeros for both mass and inertia tensor
            m_body->setinversemass(0);
            m_body->setinverseinertiatensor( _mat3() );
        } else {
            // otherwise we need to calculate the mass
            float volume = m_half_size.magnitude() * 2.0f;
            float mass   = volume / invdensity;

            m_body->setmass(mass);

            // and calculate the inertia tensor from the mass and size
            mass *= 0.333f;
            _mat3 tensor;
            tensor.setinertiatensorcoeffs(
                mass * m_half_size.y*m_half_size.y + m_half_size.z*m_half_size.z,
                mass * m_half_size.y*m_half_size.x + m_half_size.z*m_half_size.z,
                mass * m_half_size.y*m_half_size.x + m_half_size.z*m_half_size.y
                );
            m_body->setinertiatensor(tensor);
        }

    }

    /**
     * performs the division of the given block into four, writing the
     * eight new blocks into the given blocks array. the blocks array can be
     * a pointer to the same location as the target pointer: since the
     * original block is always deleted, this effectively reuses its storage.
     * the algorithm is structured to allow this reuse.
     */
    void divideblock(const contact& contact, block* target, block* blocks) {
        // find out if we're block one or two in the contact structure, and
        // therefore what the contact normal is.
        _vec3 normal = contact.m_contact_normal;
        rigid_body *body = contact.m_body[0];
        if (body != target->m_body) {
            normal.invert();
            body = contact.m_body[1];
        }

        // work out where on the body (in body coordinates) the contact is
        // and its direction.
        _vec3 point = body->getpointinlocalspace(contact.m_contact_point);
        normal = body->getdirectioninlocalspace(normal);

        // work out the centre of the split: this is the point coordinates
        // for each of the axes perpendicular to the normal, and 0 for the
        // axis along the normal.
        point = point - normal * (point * normal);

        // take a copy of the half size, so we can create the new blocks.
        _vec3 size = target->m_half_size;

        // take a copy also of the body's other data.
        rigid_body tempbody;
        tempbody.setposition(body->getposition());
        tempbody.setorientation(body->getorientation());
        tempbody.setvelocity(body->getvelocity());
        tempbody.setrotation(body->getrotation());
        tempbody.setlineardamping(body->getlineardamping());
        tempbody.setangulardamping(body->getangulardamping());
        tempbody.setinverseinertiatensor(body->getinverseinertiatensor());
        tempbody.calculatederiveddata();

        // remove the old block
        target->m_exists = false;

        // work out the inverse density of the old block
        float invdensity =
            m_half_size.magnitude()*8 * body->getinversemass();

        // now split the block into eight.
        for (unsigned i = 0; i < 8; i++) {
            // find the minimum and maximum extents of the new block
            _vec3 min, max;
            if ((i & 1) == 0) {
                min.x = -size.x;
                max.x = point.x;
            } else {
                min.x = point.x;
                max.x = size.x;
            }
            if ((i & 2) == 0) {
                min.y = -size.y;
                max.y = point.y;
            } else {
                min.y = point.y;
                max.y = size.y;
            }
            if ((i & 4) == 0) {
                min.z = -size.z;
                max.z = point.z;
            } else {
                min.z = point.z;
                max.z = size.z;
            }

            // get the origin and half size of the block, in old-body
            // local coordinates.
            _vec3 halfsize = (max - min) * 0.5f;
            _vec3 newpos   = halfsize + min;

            // convert the origin to world coordinates.
            newpos = tempbody.getpointinworldspace(newpos);

            // work out the direction to the impact.
            _vec3 direction = newpos - contact.m_contact_point;
            direction.normalise();

            // set the body's properties (we assume the block has a body
            // already that we're going to overwrite).
            blocks[i].m_body->setposition(newpos);
            blocks[i].m_body->setvelocity(tempbody.getvelocity() + direction * 10.0f);
            blocks[i].m_body->setorientation(tempbody.getorientation());
            blocks[i].m_body->setrotation(tempbody.getrotation());
            blocks[i].m_body->setlineardamping(tempbody.getlineardamping());
            blocks[i].m_body->setangulardamping(tempbody.getangulardamping());
            blocks[i].m_body->setawake(true);
            blocks[i].m_body->setacceleration( _utility::gravity );
            blocks[i].m_body->clearaccumulators();
            blocks[i].m_body->calculatederiveddata();
            blocks[i].m_offset = _mat4();
            blocks[i].m_exists = true;
            blocks[i].m_half_size = halfsize;

            // finally calculate the mass and inertia tensor of the new block
            blocks[i].calculatemassproperties(invdensity);
        }
    }
};


struct fracture : public application_object {

	_string m_state_string;

    /** holds the maximum number of contacts. */
    const static unsigned maxcontacts = 256;

    /** holds the array of contacts. */
    contact m_contacts[maxcontacts];

    /** holds the collision data structure for collision detection. */
    collision_data m_cdata;

    /** holds the contact resolver. */
    contact_resolver m_resolver;

    /** true if the contacts should be rendered. */
    bool m_render_debug_info;

	bool m_pause_simulation;

    /** pauses the simulation after the next frame automatically */
    bool m_auto_pause_simulation;

    /** processes the contact generation code. */
    void generatecontacts();

    /** processes the objects in the simulation forward in time. */
    void updateobjects( float duration);

    void reset();


	random m_random;

    /** tracks if a block has been hit. */
    bool m_hit;
    bool m_ball_active;
    unsigned m_fracture_contact;


    /** holds the bodies. */
    block m_blocks[MAX_BLOCKS];

    /** holds the projectile. */
    collision_sphere m_ball;

	/**
	* vertex buffers for drawing points lines (ground)
	*/
	IDirect3DVertexBuffer9* m_buffer;
	IDirect3DVertexBuffer9* m_point_buffer;

	fracture();

	virtual bool init();
	virtual void clear();
	virtual bool update();

	virtual bool render();

	/**
	* finishes drawing the frame, adding debugging information
	* as needed.
	*/
	bool drawdebug();

	virtual _string  getstatestring() { return m_state_string; }

	virtual void key(uint8_t key);

};
