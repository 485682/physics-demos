#include "collide_fine.h"

#include <memory.h>
#include <cstdlib>
#include <cstdio>

void collision_primitive::calculateinternals() {
    m_transform = m_body->gettransform() * m_offset;
}

bool intersection_tests::sphereandhalfspace( const collision_sphere &sphere, const collision_plane &plane) {
    // find the distance from the origin
    float balldistance =_dot( plane.m_direction,sphere.getaxis(3) ) - sphere.m_radius;

    // check for the intersection
    return balldistance <= plane.m_offset;
}

bool intersection_tests::sphereandsphere( const collision_sphere &one, const collision_sphere &two) {
    // find the vector between the objects
    _vec3 midline = one.getaxis(3) - two.getaxis(3);

    // see if it is large enough.
    return midline.squaremagnitude() < (one.m_radius+two.m_radius)*(one.m_radius+two.m_radius);
}

static inline float transformtoaxis( const collision_box &box, const _vec3 &axis ) {
    return
        box.m_half_size.x * abs( _dot( axis , box.getaxis(0) ) ) +
        box.m_half_size.y * abs( _dot( axis , box.getaxis(1) ) ) +
        box.m_half_size.z * abs( _dot( axis , box.getaxis(2) ) );
}

/**
 * this function checks if the two boxes overlap
 * along the given axis. the final parameter tocentre
 * is used to pass in the vector between the boxes centre
 * points, to avoid having to recalculate it each time.
 */
static inline bool overlaponaxis(
    const collision_box &one,
    const collision_box &two,
    const _vec3 &axis,
    const _vec3 &tocentre
    )
{
    // project the half-size of one onto axis
    float oneproject = transformtoaxis(one, axis);
    float twoproject = transformtoaxis(two, axis);

    // project this onto the axis
    float distance = abs( _dot(tocentre , axis) );

    // check for overlap
    return (distance < oneproject + twoproject);
}

// this preprocessor definition is only used as a convenience
// in the boxandbox intersection  method.
#define test_overlap(axis) overlaponaxis(one, two, (axis), tocentre)

bool intersection_tests::boxandbox(
	const collision_box &one,
	const collision_box &two
	)
{
	// find the vector between the two centres
	_vec3 tocentre = two.getaxis(3) - one.getaxis(3);

	return (
		// check on box one's axes first
		test_overlap(one.getaxis(0)) &&
		test_overlap(one.getaxis(1)) &&
		test_overlap(one.getaxis(2)) &&

		// and on two's
		test_overlap(two.getaxis(0)) &&
		test_overlap(two.getaxis(1)) &&
		test_overlap(two.getaxis(2)) &&

		// now on the cross products
		test_overlap( _cross( one.getaxis(0) , two.getaxis(0) ) ) &&
		test_overlap( _cross( one.getaxis(0) , two.getaxis(1) ) ) &&
		test_overlap( _cross( one.getaxis(0) , two.getaxis(2) ) ) &&
		test_overlap( _cross( one.getaxis(1) , two.getaxis(0) ) ) &&
		test_overlap( _cross( one.getaxis(1) , two.getaxis(1) ) ) &&
		test_overlap( _cross( one.getaxis(1) , two.getaxis(2) ) ) &&
		test_overlap( _cross( one.getaxis(2) , two.getaxis(0) ) ) &&
		test_overlap( _cross( one.getaxis(2) , two.getaxis(1) ) ) &&
		test_overlap( _cross( one.getaxis(2) , two.getaxis(2) ) )
		);
}
#undef test_overlap

bool intersection_tests::boxandhalfspace(
    const collision_box &box,
    const collision_plane &plane
    )
{
    // work out the projected radius of the box onto the plane direction
    float projectedradius = transformtoaxis(box, plane.m_direction);

    // work out how far the box is from the origin
    float boxdistance = _dot( plane.m_direction , box.getaxis(3) ) - projectedradius;

    // check for the intersection
    return boxdistance <= plane.m_offset;
}

uint32_t collision_detector::sphereandtrueplane(
    const collision_sphere &sphere,
    const collision_plane &plane,
    collision_data *data
    )
{
    // make sure we have contacts
	if (data->m_contacts_left <= 0) { return 0; }

    // cache the sphere position
    _vec3 position = sphere.getaxis(3);

    // find the distance from the plane
    float centredistance = _dot( plane.m_direction , position ) - plane.m_offset;

    // check if we're within radius
    if (centredistance*centredistance > sphere.m_radius*sphere.m_radius) { return 0; }

    // check which side of the plane we're on
    _vec3 normal = plane.m_direction;
    float penetration = -centredistance;
    if (centredistance < 0) {
        normal *= -1;
        penetration = -penetration;
    }
    penetration += sphere.m_radius;

    // create the contact - it has a normal in the plane direction.
    contact* contact = data->m_contacts;
    contact->m_contact_normal = normal;
    contact->m_penetration = penetration;
    contact->m_contact_point = position - plane.m_direction * centredistance;
    contact->setbodydata(sphere.m_body, NULL,data->m_friction, data->m_restitution);

    data->addcontacts(1);
    return 1;
}

uint32_t collision_detector::sphereandhalfspace(
    const collision_sphere &sphere,
    const collision_plane &plane,
    collision_data *data
    )
{
    // make sure we have contacts
	if (data->m_contacts_left <= 0) { return 0; }

    // cache the sphere position
    _vec3 position = sphere.getaxis(3);

    // find the distance from the plane
    float balldistance = _dot( plane.m_direction , position) - sphere.m_radius - plane.m_offset;

	if (balldistance >= 0) { return 0; }

    // create the contact - it has a normal in the plane direction.
    contact* contact = data->m_contacts;
    contact->m_contact_normal = plane.m_direction;
    contact->m_penetration = -balldistance;
    contact->m_contact_point =  position - plane.m_direction * (balldistance + sphere.m_radius);
    contact->setbodydata(sphere.m_body, NULL, data->m_friction, data->m_restitution);

    data->addcontacts(1);
    return 1;
}

uint32_t collision_detector::sphereandsphere(
    const collision_sphere &one,
    const collision_sphere &two,
    collision_data *data
    )
{
    // make sure we have contacts
	if (data->m_contacts_left <= 0) { return 0; }

    // cache the sphere positions
    _vec3 positionone = one.getaxis(3);
    _vec3 positiontwo = two.getaxis(3);

    // find the vector between the objects
    _vec3 midline = positionone - positiontwo;
    float size = midline.magnitude();

    // see if it is large enough.
    if (size <= 0.0f || size >= one.m_radius+two.m_radius) { return 0; }

    // we manually create the normal, because we have the
    // size to hand.
    _vec3 normal = midline * (1.0f/size);

    contact* contact = data->m_contacts;
    contact->m_contact_normal = normal;
    contact->m_contact_point = positionone + midline * 0.5f;
    contact->m_penetration = (one.m_radius+two.m_radius - size);
    contact->setbodydata(one.m_body, two.m_body,data->m_friction, data->m_restitution);

    data->addcontacts(1);
    return 1;
}




/*
 * this function checks if the two boxes overlap
 * along the given axis, returning the ammount of overlap.
 * the final parameter tocentre
 * is used to pass in the vector between the boxes centre
 * points, to avoid having to recalculate it each time.
 */
static inline float penetrationonaxis(
    const collision_box &one,
    const collision_box &two,
    const _vec3 &axis,
    const _vec3 &tocentre
    )
{
    // project the half-size of one onto axis
    float oneproject = transformtoaxis(one, axis);
    float twoproject = transformtoaxis(two, axis);

    // project this onto the axis
    float distance = abs( _dot( tocentre , axis ) );

    // return the overlap (i.e. positive indicates
    // overlap, negative indicates separation).
    return oneproject + twoproject - distance;
}


static inline bool tryaxis(
    const collision_box &one,
    const collision_box &two,
    _vec3 axis,
    const _vec3& tocentre,
    uint32_t index,

    // these values may be updated
    float& smallestpenetration,
    uint32_t &smallestcase
    )
{
    // make sure we have a normalized axis, and don't check almost parallel axes
	if (axis.squaremagnitude() < 0.0001f) { return true; }
    axis.normalise();

    float penetration = penetrationonaxis(one, two, axis, tocentre);

	if (penetration < 0) { return false; }
    if (penetration < smallestpenetration) {
        smallestpenetration = penetration;
        smallestcase = index;
    }
    return true;
}

void fillpointfaceboxbox(
    const collision_box &one,
    const collision_box &two,
    const _vec3 &tocentre,
    collision_data *data,
    uint32_t best,
    float pen
    )
{
    // this method is called when we know that a vertex from
    // box two is in contact with box one.

    contact* contact = data->m_contacts;

    // we know which axis the collision is on (i.e. best),
    // but we need to work out which of the two faces on
    // this axis.
    _vec3 normal = one.getaxis(best);
    if ( _dot( one.getaxis(best) , tocentre ) > 0) { normal = normal * -1.0f; }

    // work out which vertex of box two we're colliding with.
    // using tocentre doesn't work!
    _vec3 vertex = two.m_half_size;
    if ( _dot( two.getaxis(0) , normal ) < 0) { vertex.x = -vertex.x; }
    if ( _dot( two.getaxis(1) , normal ) < 0) { vertex.y = -vertex.y; }
	if ( _dot( two.getaxis(2) , normal ) < 0) { vertex.z = -vertex.z; }

    // create the contact data
    contact->m_contact_normal = normal;
    contact->m_penetration = pen;
    contact->m_contact_point = two.gettransform() * vertex;
    contact->setbodydata(one.m_body, two.m_body,data->m_friction, data->m_restitution);
}

static inline _vec3 contactpoint(
    const _vec3 &pone,
    const _vec3 &done,
    float onesize,
    const _vec3 &ptwo,
    const _vec3 &dtwo,
    float twosize,

    // if this is true, and the contact point is outside
    // the edge (in the case of an edge-face contact) then
    // we use one's midpoint, otherwise we use two's.
    bool useone)
{
    _vec3 tost, cone, ctwo;
    float dpstaone, dpstatwo, dponetwo, smone, smtwo;
    float denom, mua, mub;

    smone = done.squaremagnitude();
    smtwo = dtwo.squaremagnitude();
    dponetwo = _dot( dtwo , done );

    tost = pone - ptwo;
    dpstaone = _dot( done , tost );
    dpstatwo = _dot( dtwo , tost );

    denom = smone * smtwo - dponetwo * dponetwo;

    // zero denominator indicates parrallel lines
    if ( abs(denom) < 0.0001f ) { return useone?pone:ptwo; }

    mua = (dponetwo * dpstatwo - smtwo * dpstaone) / denom;
    mub = (smone * dpstatwo - dponetwo * dpstaone) / denom;

    // if either of the edges has the nearest point out
    // of bounds, then the edges aren't crossed, we have
    // an edge-face contact. our point is on the edge, which
    // we know from the useone parameter.
    if (mua > onesize ||
        mua < -onesize ||
		mub > twosize ||
		mub < -twosize)
	{
		return useone?pone:ptwo;
	}
	else {
		cone = pone + done * mua;
		ctwo = ptwo + dtwo * mub;

		return cone * 0.5 + ctwo * 0.5;
	}
}

// this preprocessor definition is only used as a convenience
// in the boxandbox contact generation method.
#define _check_overlap(axis, index) \
	if (!tryaxis(one, two, (axis), tocentre, (index), pen, best)) { return 0; }

uint32_t collision_detector::boxandbox(
    const collision_box &one,
    const collision_box &two,
    collision_data *data
    )
{
    //if (!intersection_tests::boxandbox(one, two)) return 0;

    // find the vector between the two centres
    _vec3 tocentre = two.getaxis(3) - one.getaxis(3);

    // we start assuming there is no contact
    float pen = FLT_MAX;
    uint32_t best = 0xffffff;

    // now we check each axes, returning if it gives us
    // a separating axis, and keeping track of the axis with
    // the smallest penetration otherwise.
    _check_overlap(one.getaxis(0), 0);
    _check_overlap(one.getaxis(1), 1);
    _check_overlap(one.getaxis(2), 2);

    _check_overlap(two.getaxis(0), 3);
    _check_overlap(two.getaxis(1), 4);
    _check_overlap(two.getaxis(2), 5);

    // store the best axis-major, in case we run into almost
    // parallel edge collisions later
    uint32_t bestsingleaxis = best;

    _check_overlap( _cross( one.getaxis(0) , two.getaxis(0) ), 6);
    _check_overlap( _cross( one.getaxis(0) , two.getaxis(1) ), 7);
    _check_overlap( _cross( one.getaxis(0) , two.getaxis(2) ), 8);
    _check_overlap( _cross( one.getaxis(1) , two.getaxis(0) ), 9);
    _check_overlap( _cross( one.getaxis(1) , two.getaxis(1) ), 10);
    _check_overlap( _cross( one.getaxis(1) , two.getaxis(2) ), 11);
    _check_overlap( _cross( one.getaxis(2) , two.getaxis(0) ), 12);
    _check_overlap( _cross( one.getaxis(2) , two.getaxis(1) ), 13);
    _check_overlap( _cross( one.getaxis(2) , two.getaxis(2) ), 14);

    // make sure we've got a result.
	if(best == 0xffffff){ application_error("best");  }

    // we now know there's a collision, and we know which
    // of the axes gave the smallest penetration. we now
    // can deal with it in different ways depending on
    // the case.
    if (best < 3) {
        // we've got a vertex of box two on a face of box one.
        fillpointfaceboxbox(one, two, tocentre, data, best, pen);
        data->addcontacts(1);
        return 1;
    }else if (best < 6){
        // we've got a vertex of box one on a face of box two.
        // we use the same algorithm as above, but swap around
        // one and two (and therefore also the vector between their
        // centres).
        fillpointfaceboxbox(two, one, tocentre*-1.0f, data, best-3, pen);
        data->addcontacts(1);
        return 1;
    }else {
        // we've got an edge-edge contact. find out which axes
        best -= 6;
        uint32_t oneaxisindex = best / 3;
        uint32_t twoaxisindex = best % 3;


        _vec3 oneaxis = one.getaxis(oneaxisindex);
        _vec3 twoaxis = two.getaxis(twoaxisindex);
        _vec3 axis = _cross( oneaxis , twoaxis);
        axis.normalise();

        // the axis should point from box one to box two.
        if ( _dot(axis , tocentre) > 0) axis = axis * -1.0f;

        // we have the axes, but not the edges: each axis has 4 edges parallel
        // to it, we need to find which of the 4 for each object. we do
        // that by finding the point in the centre of the edge. we know
        // its component in the direction of the box's collision axis is zero
        // (its a mid-point) and we determine which of the extremes in each
        // of the other axes is closest.
        _vec3 ptononeedge = one.m_half_size;
        _vec3 ptontwoedge = two.m_half_size;
        for (uint32_t i = 0; i < 3; i++) {
            if (i == oneaxisindex) ptononeedge[i] = 0;
			else if ( _dot( one.getaxis(i) , axis ) > 0) { ptononeedge[i] = -ptononeedge[i]; }

            if (i == twoaxisindex) ptontwoedge[i] = 0;
			else if ( _dot( two.getaxis(i) , axis ) < 0) { ptontwoedge[i] = -ptontwoedge[i]; }
        }

        // move them into world coordinates (they are already oriented
        // correctly, since they have been derived from the axes).
        ptononeedge = one.m_transform * ptononeedge;
        ptontwoedge = two.m_transform * ptontwoedge;

        // so we have a point and a direction for the colliding edges.
        // we need to find out point of closest approach of the two
        // line-segments.
        _vec3 vertex = contactpoint(
            ptononeedge, oneaxis, one.m_half_size[oneaxisindex],
            ptontwoedge, twoaxis, two.m_half_size[twoaxisindex],
            bestsingleaxis > 2
            );

        // we can fill the contact.
        contact* contact = data->m_contacts;

        contact->m_penetration = pen;
        contact->m_contact_normal = axis;
        contact->m_contact_point = vertex;
        contact->setbodydata(one.m_body, two.m_body,
            data->m_friction, data->m_restitution);

        data->addcontacts(1);
        return 1;
    }
    return 0;
}
#undef _check_overlap




uint32_t collision_detector::boxandpoint(
    const collision_box &box,
    const _vec3 &point,
    collision_data *data
    )
{
    // transform the point into box coordinates
    _vec3 relpt = box.m_transform.transforminverse(point);

    _vec3 normal;

    // check each axis, looking for the axis on which the
    // penetration is least deep.
    float min_depth = box.m_half_size.x - abs(relpt.x);
	if (min_depth < 0) { return 0; }

    normal = box.getaxis(0) * ((relpt.x < 0)?-1.0f:1.0f);

    float depth = box.m_half_size.y - abs(relpt.y);
	if (depth < 0) { return 0; }
    else if (depth < min_depth) {
        min_depth = depth;
        normal = box.getaxis(1) * ((relpt.y < 0)?-1.0f:1.0f);
    }

    depth = box.m_half_size.z - abs(relpt.z);
	if (depth < 0) { return 0; }
    else if (depth < min_depth) {
        min_depth = depth;
        normal = box.getaxis(2) * ((relpt.z < 0)?-1.0f:1.0f);
    }

    // compile the contact
    contact* contact = data->m_contacts;
    contact->m_contact_normal = normal;
    contact->m_contact_point = point;
    contact->m_penetration = min_depth;

    // note that we don't know what rigid body the point
    // belongs to, so we just use null. where this is called
    // this value can be left, or filled in.
    contact->setbodydata(box.m_body, NULL, data->m_friction, data->m_restitution);

    data->addcontacts(1);
    return 1;
}

uint32_t collision_detector::boxandsphere(
    const collision_box &box,
    const collision_sphere &sphere,
    collision_data *data
    )
{
    // transform the centre of the sphere into box coordinates
    _vec3 centre = sphere.getaxis(3);
    _vec3 relcentre = box.m_transform.transforminverse(centre);

    // early out check to see if we can exclude the contact
    if (abs(relcentre.x) - sphere.m_radius > box.m_half_size.x ||
        abs(relcentre.y) - sphere.m_radius > box.m_half_size.y ||
        abs(relcentre.z) - sphere.m_radius > box.m_half_size.z)
    {
        return 0;
    }

    _vec3 closestpt(0,0,0);
    float dist;

    // clamp each coordinate to the box.
    dist = relcentre.x;
	if (dist > box.m_half_size.x) { dist = box.m_half_size.x; }
	if (dist < -box.m_half_size.x) { dist = -box.m_half_size.x; }
    closestpt.x = dist;

    dist = relcentre.y;
	if (dist > box.m_half_size.y) { dist = box.m_half_size.y; }
	if (dist < -box.m_half_size.y) { dist = -box.m_half_size.y; }
    closestpt.y = dist;

    dist = relcentre.z;
	if (dist > box.m_half_size.z)  { dist = box.m_half_size.z; }
	if (dist < -box.m_half_size.z) { dist = -box.m_half_size.z; }
    closestpt.z = dist;

    // check we're in contact
    dist = (closestpt - relcentre).squaremagnitude();
	if (dist > sphere.m_radius * sphere.m_radius) { return 0; }

    // compile the contact
    _vec3 closestptworld = box.m_transform.transform(closestpt);

    contact* contact = data->m_contacts;
    contact->m_contact_normal = (closestptworld - centre);
    contact->m_contact_normal.normalise();


    contact->m_contact_point = closestptworld;
    contact->m_penetration = sphere.m_radius - sqrt(dist);
    contact->setbodydata(box.m_body, sphere.m_body,data->m_friction, data->m_restitution);

    data->addcontacts(1);
    return 1;
}

uint32_t collision_detector::boxandhalfspace(
    const collision_box &box,
    const collision_plane &plane,
    collision_data *data
    )
{
    // make sure we have contacts
	if (data->m_contacts_left <= 0) { return 0; }

    // check for intersection
    if (!intersection_tests::boxandhalfspace(box, plane)) { return 0; }

    // we have an intersection, so find the intersection points. we can make
    // do with only checking vertices. if the box is resting on a plane
    // or on an edge, it will be reported as four or two contact points.

    // go through each combination of + and - for each half-size
    static float mults[8][3] = {{1,1,1},{-1,1,1},{1,-1,1},{-1,-1,1},
                               {1,1,-1},{-1,1,-1},{1,-1,-1},{-1,-1,-1}};

    contact* contact = data->m_contacts;
    uint32_t contactsused = 0;
    for (uint32_t i = 0; i < 8; i++) {

        // calculate the position of each vertex
        _vec3 vertexpos(mults[i][0], mults[i][1], mults[i][2]);
        vertexpos.componentproductupdate(box.m_half_size);
        vertexpos = box.m_transform.transform(vertexpos);

        // calculate the distance from the plane
        float vertexdistance = _dot( vertexpos , plane.m_direction );

        // compare this to the plane's distance
        if (vertexdistance <= plane.m_offset) {
            // create the contact data.

            // the contact point is halfway between the vertex and the
            // plane - we multiply the direction by half the separation
            // distance and add the vertex location.
            contact->m_contact_point  = plane.m_direction;
            contact->m_contact_point *= (vertexdistance-plane.m_offset);
            contact->m_contact_point += vertexpos;
            contact->m_contact_normal = plane.m_direction;
            contact->m_penetration    = plane.m_offset - vertexdistance;
            // write the appropriate data
            contact->setbodydata(box.m_body, NULL,data->m_friction, data->m_restitution);

            // move onto the next contact
            contact++;
            contactsused++;
			if (contactsused == (uint32_t)data->m_contacts_left) { return contactsused; }
        }
    }

    data->addcontacts(contactsused);
    return contactsused;
}
