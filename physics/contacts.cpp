

#include "contacts.h"
#include <memory.h>

#include <body.h>

// contact implementation

void contact::setbodydata(rigid_body* one, rigid_body *two, float friction, float restitution) {
    m_body[0] = one;
    m_body[1] = two;
    m_friction = friction;
    m_restitution = restitution;
}

void contact::matchawakestate() {
    // collisions with the world never cause a body to wake up.
    if (!m_body[1]) { return; }

    bool body0awake = m_body[0]->getawake();
    bool body1awake = m_body[1]->getawake();

    // wake up only the sleeping one
    if (body0awake ^ body1awake) {
		if (body0awake) { m_body[1]->setawake(); }
		else { m_body[0]->setawake(); }
    }
}

/*
 * swaps the bodies in the current contact, so body 0 is at body 1 and
 * vice versa. this also changes the direction of the contact normal,
 * but doesn't update any calculated internal data. if you are calling
 * this method manually, then call calculateinternals afterwards to
 * make sure the internal data is up to date.
 */
void contact::swapbodies() {

	m_contact_normal *= -1;

    rigid_body *temp = m_body[0];
    m_body[0] = m_body[1];
    m_body[1] = temp;
}

/*
 * constructs an arbitrary orthonormal basis for the contact.  this is
 * stored as a 3x3 matrix, where each vector is a column (in other
 * words the matrix transforms contact space into world space). the x
 * direction is generated from the contact normal, and the y and z
 * directionss are set so they are at right angles to it.
 */
inline
void contact::calculatecontactbasis() {

    _vec3 contacttangent[2];

    // check whether the z-axis is nearer to the x or y axis
    if ( abs(m_contact_normal.x) > abs(m_contact_normal.y) ) {
        // scaling factor to ensure the results are normalised
        const float s = (float)1.0/sqrt(m_contact_normal.z * m_contact_normal.z + m_contact_normal.x*m_contact_normal.x);

        // the new x-axis is at right angles to the world y-axis
        contacttangent[0].x =  m_contact_normal.z*s;
        contacttangent[0].y =  0;
        contacttangent[0].z = -m_contact_normal.x*s;

        // the new y-axis is at right angles to the new x- and z- axes
        contacttangent[1].x =  m_contact_normal.y*contacttangent[0].x;
        contacttangent[1].y =  m_contact_normal.z*contacttangent[0].x - m_contact_normal.x*contacttangent[0].z;
        contacttangent[1].z = -m_contact_normal.y*contacttangent[0].x;
	} else {
        // scaling factor to ensure the results are normalised
        const float s = (float)1.0/sqrt( m_contact_normal.z*m_contact_normal.z + m_contact_normal.y*m_contact_normal.y);

        // the new x-axis is at right angles to the world x-axis
        contacttangent[0].x =  0;
        contacttangent[0].y = -m_contact_normal.z*s;
        contacttangent[0].z =  m_contact_normal.y*s;

        // the new y-axis is at right angles to the new x- and z- axes
        contacttangent[1].x =  m_contact_normal.y*contacttangent[0].z - m_contact_normal.z*contacttangent[0].y;
        contacttangent[1].y = -m_contact_normal.x*contacttangent[0].z;
        contacttangent[1].z =  m_contact_normal.x*contacttangent[0].y;
    }

    // make a matrix from the three vectors.
    m_contact_to_world.setcomponents(
        m_contact_normal,
        contacttangent[0],
        contacttangent[1]);
}

_vec3 contact::calculatelocalvelocity(uint32_t bodyindex, float duration) {

	rigid_body *thisbody = m_body[bodyindex];

    // work out the velocity of the contact point.
    _vec3 velocity = _cross( thisbody->getrotation() , m_relative_contact_position[bodyindex] );
    velocity += thisbody->getvelocity();

    // turn the velocity into contact-coordinates.
	_vec3 contactvelocity = m_contact_to_world.transformtranspose(velocity);

    // calculate the ammount of velocity that is due to forces without
    // reactions.
    _vec3 accvelocity = thisbody->getlastframeacceleration() * duration;

    // calculate the velocity in contact-coordinates.
    accvelocity = m_contact_to_world.transformtranspose(accvelocity);

    // we ignore any component of acceleration in the contact normal
    // direction, we are only interested in planar acceleration
    accvelocity.x = 0;

    // add the planar velocities - if there's enough friction they will
    // be removed during velocity resolution
    contactvelocity += accvelocity;

    // and return it
    return contactvelocity;
}


void contact::calculatedesireddeltavelocity(float duration) {

	const static float velocitylimit = 0.25f;

    // calculate the acceleration induced velocity accumulated this frame
    float velocityfromacc = 0;

    if (m_body[0]->getawake()) {
		velocityfromacc += _dot( (m_body[0]->getlastframeacceleration()*duration) , m_contact_normal );
	}
    if ( m_body[1] && m_body[1]->getawake() ){
        velocityfromacc -= _dot( (m_body[1]->getlastframeacceleration() * duration), m_contact_normal);
    }

    // if the velocity is very slow, limit the restitution
    float thisrestitution = m_restitution;
    if ( abs(m_contact_velocity.x) < velocitylimit) { thisrestitution = 0.0f; }

    // combine the bounce velocity with the removed
    // acceleration velocity.
    m_desired_delta_velocity = -m_contact_velocity.x -thisrestitution * (m_contact_velocity.x - velocityfromacc);
}


void contact::calculateinternals( float duration) {

	// check if the first object is null, and swap if it is.
	if (!m_body[0]) { swapbodies(); }

	if(!m_body[0]){ application_error("null ptr"); }

    // calculate an set of axis at the contact point.
    calculatecontactbasis();

    // store the relative position of the contact relative to each body
    m_relative_contact_position[0] = m_contact_point - m_body[0]->getposition();
    if (m_body[1]) {
        m_relative_contact_position[1] = m_contact_point - m_body[1]->getposition();
    }

    // find the relative velocity of the bodies at the contact point.
    m_contact_velocity = calculatelocalvelocity(0, duration);
    if (m_body[1]) {
        m_contact_velocity -= calculatelocalvelocity(1, duration);
    }

    // calculate the desired change in velocity for resolution
    calculatedesireddeltavelocity(duration);
}

void contact::applyvelocitychange ( _vec3 velocitychange[2] , _vec3 rotationchange[2]) {
    // get hold of the inverse mass and inverse inertia tensor, both in
    // world coordinates.
    _mat3 inverseinertiatensor[2];
    m_body[0]->getinverseinertiatensorworld(&inverseinertiatensor[0]);
    if (m_body[1]) { m_body[1]->getinverseinertiatensorworld(&inverseinertiatensor[1]); }

    // we will calculate the impulse for each contact axis
    _vec3 impulsecontact;

    if (m_friction == 0.0f) {
        // use the short format for frictionless contacts
        impulsecontact = calculatefrictionlessimpulse(inverseinertiatensor);
    }else {
        // otherwise we may have impulses that aren't in the direction of the
        // contact, so we need the more complex version.
        impulsecontact = calculatefrictionimpulse(inverseinertiatensor);
    }

    // convert impulse to world coordinates
    _vec3 impulse = m_contact_to_world.transform(impulsecontact);

    // split in the impulse into linear and rotational components
    _vec3 impulsivetorque = _cross(m_relative_contact_position[0] , impulse );
    rotationchange[0] = inverseinertiatensor[0].transform(impulsivetorque);
    velocitychange[0].clear();
    velocitychange[0].addscaledvector(impulse, m_body[0]->getinversemass());

    // apply the changes
    m_body[0]->addvelocity(velocitychange[0]);
    m_body[0]->addrotation(rotationchange[0]);

    if (m_body[1])
    {
        // work out body one's linear and angular changes
        _vec3 impulsivetorque = _cross( impulse , m_relative_contact_position[1]);
        rotationchange[1] = inverseinertiatensor[1].transform(impulsivetorque);
        velocitychange[1].clear();
        velocitychange[1].addscaledvector(impulse, -m_body[1]->getinversemass());

        // and apply them.
        m_body[1]->addvelocity(velocitychange[1]);
        m_body[1]->addrotation(rotationchange[1]);
    }
}

inline
_vec3 contact::calculatefrictionlessimpulse( _mat3 * inverseinertiatensor) {
    _vec3 impulsecontact;

    // build a vector that shows the change in velocity in
    // world space for a unit impulse in the direction of the contact
    // normal.
    _vec3 deltavelworld = _cross(m_relative_contact_position[0] , m_contact_normal);
    deltavelworld = inverseinertiatensor[0].transform(deltavelworld);
    deltavelworld = _cross(deltavelworld , m_relative_contact_position[0]);

    // work out the change in velocity in contact coordiantes.
    float deltavelocity = _dot( deltavelworld , m_contact_normal);

    // add the linear component of velocity change
    deltavelocity += m_body[0]->getinversemass();

    // check if we need to the second body's data
    if (m_body[1]) {

        // go through the same transformation sequence again
        _vec3 deltavelworld = _cross( m_relative_contact_position[1] , m_contact_normal);
        deltavelworld = inverseinertiatensor[1].transform(deltavelworld);
        deltavelworld = _cross( deltavelworld , m_relative_contact_position[1]);

        // add the change in velocity due to rotation
        deltavelocity += _dot( deltavelworld , m_contact_normal);

        // add the change in velocity due to linear motion
        deltavelocity += m_body[1]->getinversemass();
    }

    // calculate the required size of the impulse
    impulsecontact.x = m_desired_delta_velocity / deltavelocity;
    impulsecontact.y = 0;
    impulsecontact.z = 0;
    return impulsecontact;
}

inline
_vec3 contact::calculatefrictionimpulse( _mat3 * inverseinertiatensor) {
    _vec3 impulsecontact;
    float inversemass = m_body[0]->getinversemass();

    // the equivalent of a cross product in matrices is multiplication
    // by a skew symmetric matrix - we build the matrix for converting
    // between linear and angular quantities.
    _mat3 impulsetotorque;
    impulsetotorque.setskewsymmetric(m_relative_contact_position[0]);

    // build the matrix to convert contact impulse to change in velocity
    // in world coordinates.
    _mat3 deltavelworld = impulsetotorque;
    deltavelworld *= inverseinertiatensor[0];
    deltavelworld *= impulsetotorque;
    deltavelworld *= -1;

    // check if we need to add body two's data
    if (m_body[1]) {
        // set the cross product matrix
        impulsetotorque.setskewsymmetric(m_relative_contact_position[1]);

        // calculate the velocity change matrix
		_mat3 deltavelworld2 = impulsetotorque;
		deltavelworld2 *= inverseinertiatensor[1];
		deltavelworld2 *= impulsetotorque;
		deltavelworld2 *= -1;

        // add to the total delta velocity.
        deltavelworld += deltavelworld2;

        // add to the inverse mass
        inversemass += m_body[1]->getinversemass();
    }

    // do a change of basis to convert into contact coordinates.
    _mat3 deltavelocity = _transpose( m_contact_to_world );
    deltavelocity *= deltavelworld;
    deltavelocity *= m_contact_to_world;

    // add in the linear velocity change
    deltavelocity[0][0] += inversemass;
    deltavelocity[1][1] += inversemass;
    deltavelocity[2][2] += inversemass;

    // invert to get the impulse needed per unit velocity
    _mat3 impulsematrix = deltavelocity.inverse();

    // find the target velocities to kill
    _vec3 velkill( m_desired_delta_velocity, -m_contact_velocity.y, -m_contact_velocity.z );

    // find the impulse to kill target velocities
    impulsecontact = impulsematrix.transform(velkill);

    // check for exceeding friction
    float planarimpulse = sqrt(
        impulsecontact.y*impulsecontact.y +
        impulsecontact.z*impulsecontact.z
        );
    if (planarimpulse > impulsecontact.x * m_friction) {
        // we need to use dynamic friction
        impulsecontact.y /= planarimpulse;
        impulsecontact.z /= planarimpulse;

        impulsecontact.x = 
			deltavelocity[0][0] +
            deltavelocity[1][0] * m_friction * impulsecontact.y +
            deltavelocity[2][0] * m_friction * impulsecontact.z;
        impulsecontact.x   = m_desired_delta_velocity / impulsecontact.x;
        impulsecontact.y  *= m_friction * impulsecontact.x;
        impulsecontact.z  *= m_friction * impulsecontact.x;
    }
    return impulsecontact;
}

void contact::applypositionchange( _vec3 linearchange[2], _vec3 angularchange[2], float penetration) {

	
	const float angularlimit = 0.2f;
	float angularmove[2];
	float linearmove[2];

	float totalinertia = 0;
	float linearinertia[2];
	float angularinertia[2];

	// we need to work out the inertia of each object in the direction
	// of the contact normal, due to angular inertia only.
	for (uint32_t i = 0; i < 2; i++) {

		if (m_body[i]) {
			_mat3 inverseinertiatensor;
			m_body[i]->getinverseinertiatensorworld(&inverseinertiatensor);

			// use the same procedure as for calculating frictionless
			// velocity change to work out the angular inertia.
			_vec3 angularinertiaworld = _cross( m_relative_contact_position[i] , m_contact_normal);
			angularinertiaworld = inverseinertiatensor.transform(angularinertiaworld);
			angularinertiaworld = _cross( angularinertiaworld , m_relative_contact_position[i]);
			angularinertia[i]   = _dot( angularinertiaworld , m_contact_normal );

			// the linear component is simply the inverse mass
			linearinertia[i] = m_body[i]->getinversemass();

			// keep track of the total inertia from all components
			totalinertia += linearinertia[i] + angularinertia[i];

			// we break the loop here so that the totalinertia value is
			// completely calculated (by both iterations) before
			// continuing.
		}

	}

	// loop through again calculating and applying the changes
	for (uint32_t i = 0; i < 2; i++) {

		if (m_body[i]) {
			// the linear and angular movements required are in proportion to
			// the two inverse inertias.
			float sign = (i == 0)?1.0f:-1.0f;
			angularmove[i] = sign * penetration * (angularinertia[i] / totalinertia);
			linearmove[i]  = sign * penetration * (linearinertia[i] / totalinertia);

			// to avoid angular projections that are too great (when mass is large
			// but inertia tensor is small) limit the angular move.
			_vec3 projection = m_relative_contact_position[i];
			projection.addscaledvector( m_contact_normal, -(_dot( m_relative_contact_position[i] , m_contact_normal)) );

			// use the small angle approximation for the sine of the angle (i.e.
			// the magnitude would be sine(angularlimit) * projection.magnitude
			// but we approximate sine(angularlimit) to angularlimit).
			float maxmagnitude = angularlimit * projection.magnitude();

			if (angularmove[i] < -maxmagnitude) {
				float totalmove = angularmove[i] + linearmove[i];
				angularmove[i] = -maxmagnitude;
				linearmove[i] = totalmove - angularmove[i];
			}else if (angularmove[i] > maxmagnitude) {
				float totalmove = angularmove[i] + linearmove[i];
				angularmove[i] = maxmagnitude;
				linearmove[i] = totalmove - angularmove[i];
			}

			// we have the linear amount of movement required by turning
			// the rigid body (in angularmove[i]). we now need to
			// calculate the desired rotation to achieve that.
			if (angularmove[i] == 0) {
				// easy case - no angular movement means no rotation.
				angularchange[i].clear();
			}else {
				// work out the direction we'd like to rotate in.
				_vec3 targetangulardirection =_cross( m_relative_contact_position[i] , m_contact_normal );

				_mat3 inverseinertiatensor;
				m_body[i]->getinverseinertiatensorworld(&inverseinertiatensor);

				// work out the direction we'd need to rotate to achieve that
				angularchange[i] = inverseinertiatensor.transform(targetangulardirection) * (angularmove[i] / angularinertia[i]);
			}

			// velocity change is easier - it is just the linear movement
			// along the contact normal.
			linearchange[i] = m_contact_normal * linearmove[i];

			// now we can start to apply the values we've calculated.
			// apply the linear movement
			_vec3 pos;
			m_body[i]->getposition(&pos);
			pos.addscaledvector(m_contact_normal, linearmove[i]);
			m_body[i]->setposition(pos);

			// and the change in orientation
			_quaternion q;
			m_body[i]->getorientation(&q);
			q.addscaledvector( angularchange[i] , 1.0f );
			m_body[i]->setorientation(q);

			// we need to calculate the derived data for any body that is
			// asleep, so that the changes are reflected in the object's
			// data. otherwise the resolution will not change the position
			// of the object, and the next collision detection round will
			// have the same penetration.
			if (!m_body[i]->getawake()) { m_body[i]->calculatederiveddata(); }
		}
	}
}

// contact resolver implementation

contact_resolver::contact_resolver(
	uint32_t iterations, 
	float velocityepsilon, 
	float positionepsilon
	) 
{
		setiterations(iterations, iterations);
		setepsilon(velocityepsilon, positionepsilon);
}

contact_resolver::contact_resolver(
	uint32_t velocityiterations,
	uint32_t positioniterations,
	float velocityepsilon,
	float positionepsilon
	)
{
	setiterations(velocityiterations);
	setepsilon(velocityepsilon, positionepsilon);
}

void contact_resolver::setiterations(uint32_t iterations) { setiterations(iterations, iterations); }

void contact_resolver::setiterations(uint32_t velocityiterations,  uint32_t positioniterations) {
    m_velocity_iterations = velocityiterations;
    m_position_iterations = positioniterations;
}

void contact_resolver::setepsilon(float velocityepsilon,float positionepsilon){
    m_velocity_epsilon = velocityepsilon;
    m_position_epsilon = positionepsilon;
}

void contact_resolver::resolvecontacts(contact *contacts, uint32_t numcontacts,float duration){
    // make sure we have something to do.
	if (numcontacts == 0) { return; }
	if (!isvalid()) { return; }


    // prepare the contacts for processing
    preparecontacts(contacts, numcontacts, duration);

    // resolve the interpenetration problems with the contacts.
    adjustpositions(contacts, numcontacts, duration);

    // resolve the velocity problems with the contacts.
    adjustvelocities(contacts, numcontacts, duration);

}

void contact_resolver::preparecontacts(contact* contacts,uint32_t numcontacts,float duration) {
    // generate contact velocity and axis information.
    contact* lastcontact = contacts + numcontacts;
    for (contact* contact_=contacts; contact_ < lastcontact; contact_++) {
        // calculate the internal contact data (inertia, basis, etc).
        contact_->calculateinternals(duration);
    }
}

void contact_resolver::adjustvelocities(contact *c, uint32_t numcontacts, float duration) {

	_vec3 velocitychange[2], rotationchange[2];
    _vec3 deltavel;

    // iteratively handle impacts in order of severity.
    m_velocity_iterations_used = 0;
    while ( m_velocity_iterations_used < m_velocity_iterations ) {
        // find contact with maximum magnitude of probable velocity change.
        float max = m_velocity_epsilon;
        uint32_t index = numcontacts;
        for (uint32_t i = 0; i < numcontacts; i++) {

            if (c[i].m_desired_delta_velocity > max) {
                max = c[i].m_desired_delta_velocity;
                index = i;
            }

        }
		if (index == numcontacts) { break; }

        // match the awake state at the contact
        c[index].matchawakestate();

        // do the resolution on the contact that came out top.
        c[index].applyvelocitychange(velocitychange, rotationchange);

        // with the change in velocity of the two bodies, the update of
        // contact velocities means that some of the relative closing
        // velocities need recomputing.
		for (uint32_t i = 0; i < numcontacts; i++) {
			// check each body in the contact
			for (uint32_t b = 0; b < 2; b++) {
				if (c[i].m_body[b]) {
					// check for a match with each body in the newly
					// resolved contact
					for (uint32_t d = 0; d < 2; d++) {

						if (c[i].m_body[b] == c[index].m_body[d]) {
							deltavel = velocitychange[d] + _cross(rotationchange[d],c[i].m_relative_contact_position[b]);

							// the sign of the change is negative if we're dealing
							// with the second body in a contact.
							c[i].m_contact_velocity += c[i].m_contact_to_world.transformtranspose(deltavel) * (b?-1.0f:1.0f);
							c[i].calculatedesireddeltavelocity(duration);
						}
					}
				}
			}
        }
        m_velocity_iterations_used++;
    }
}

void contact_resolver::adjustpositions(contact *c,uint32_t numcontacts, float duration) {

	uint32_t i,index;
    _vec3 linearchange[2], angularchange[2];
    float max;
    _vec3 deltaposition;

    // iteratively resolve interpenetrations in order of severity.
    m_position_iterations_used = 0;
    while ( m_position_iterations_used < m_position_iterations ) {
        // find biggest penetration
        max = m_position_epsilon;
        index = numcontacts;
        for (i=0; i<numcontacts; i++) {
            if (c[i].m_penetration > max) {
                max = c[i].m_penetration;
                index = i;
            }
        }
		if (index == numcontacts) { break; }

        // match the awake state at the contact
        c[index].matchawakestate();

        // resolve the penetration.
        c[index].applypositionchange(linearchange,angularchange,max);

        // again this action may have changed the penetration of other
        // bodies, so we update contacts.
        for (i = 0; i < numcontacts; i++) {
            // check each body in the contact
            for (uint32_t b = 0; b < 2; b++) if (c[i].m_body[b]) {
                // check for a match with each body in the newly
                // resolved contact
                for (uint32_t d = 0; d < 2; d++) {
                    if (c[i].m_body[b] == c[index].m_body[d]) {
                        deltaposition = linearchange[d] + _cross(angularchange[d] , c[i].m_relative_contact_position[b]);

                        // the sign of the change is positive if we're
                        // dealing with the second body in a contact
                        // and negative otherwise (because we're
                        // subtracting the resolution)..
                        c[i].m_penetration += _dot( deltaposition,c[i].m_contact_normal) * (b?1:-1);
                    }
                }
            }
        }
        m_position_iterations_used++;
    }
    
}
