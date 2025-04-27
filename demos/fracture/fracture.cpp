#include "fracture.h"


#include "clock.h"
#include "application.h"
#include "d3d_manager.h"
#include "object_manager.h"



fracture::fracture(): 
m_resolver(maxcontacts*8),
	m_render_debug_info(false),
	m_pause_simulation(true),
	m_auto_pause_simulation(false)
{

}


bool fracture::init(){

	m_cdata.m_contact_array = m_contacts;

	m_state_string = _string("reset: R run simulation: P render contacts: C advance frame: spcae");

	/* point buffer **********************************************************************/
	m_point_buffer = NULL;
	application_throw_hr(_api_manager->m_d3ddevice->CreateVertexBuffer(
		3601 * sizeof(_vec3),
		D3DUSAGE_WRITEONLY,0, D3DPOOL_MANAGED,	
		&m_point_buffer, 0));
	if(!m_point_buffer){ application_throw("point buffer"); }
	int p = 0;

	_vec3 * v = 0;
	application_throw_hr(m_point_buffer->Lock(0, 0, (void**)&v, 0));
	for( float i=0;i<20;i++){

		_vec4 position(i,0.0f,0.0f,1.0f);
		for (float d = 0; d < 360.0f ; d+= 2.0f ) {
			_vec4 pos = _rotate( float(_radians(d)) ,_vec3(0.0f,1.0f,0.0f)) *position;
			v[p++]  = _vec3(pos.x,pos.y, pos.z);/*2*/
		}
	}
	application_throw_hr(m_point_buffer->Unlock()); 
	/*************************************************************************************/

	/*line buffer*************************************************************************/
	m_buffer = NULL;
	application_throw_hr(_api_manager->m_d3ddevice->CreateVertexBuffer(
		4 * sizeof(_vec3),
		D3DUSAGE_WRITEONLY,0, D3DPOOL_MANAGED,	
		&m_buffer, 0));
	if(!m_buffer){ application_throw("buffer"); }
	/*************************************************************************************/


	// create the ball.
	m_ball.m_body = new rigid_body();
	m_ball.m_radius = 0.25f;
	m_ball.m_body->setmass(5.0f);
	m_ball.m_body->setdamping(0.9f, 0.9f);
	_mat3 it;
	it.setdiagonal(5.0f, 5.0f, 5.0f);
	m_ball.m_body->setinertiatensor(it);
	m_ball.m_body->setacceleration( _utility::gravity );

    m_ball.m_body->setcansleep(false);
    m_ball.m_body->setawake(true);

	_application->addflags(application_spherical_cam);

	/* camera*/
	_application->m_distance = 35.0f;
	_application->m_pitch        = -35.0f;
	_application->m_pitch_pos    = -35.0f;

    // set up the initial block
    reset();

	return true;
}

void fracture::generatecontacts() {

    m_hit = false;

    // create the ground plane data
    collision_plane plane;
    plane.m_direction = _vec3(0,1,0);
    plane.m_offset = 0;

    // set up the collision data structure
    m_cdata.reset(maxcontacts);
    m_cdata.m_friction    = 0.9f;
    m_cdata.m_restitution = 0.2f;
    m_cdata.m_tolerance   = 0.1f;

    // perform collision detection
    _mat4 transform, othertransform;
    _vec3 position, otherposition;

    for (block *block_ = m_blocks; block_ < m_blocks+MAX_BLOCKS; block_++) {
        
		if (!block_->m_exists)  { continue; }

        // check for collisions with the ground plane
		if (!m_cdata.hasmorecontacts()) { return; }
        collision_detector::boxandhalfspace(*block_, plane, &m_cdata);

        if (m_ball_active) {
            // and with the sphere
			if (!m_cdata.hasmorecontacts()) { return; }
            if (collision_detector::boxandsphere(*block_, m_ball, &m_cdata)) {
                m_hit = true;
                m_fracture_contact = m_cdata.m_contact_count-1;
            }
        }

        // check for collisions with each other box
        for (block *other = block_+1; other < m_blocks+MAX_BLOCKS; other++) {
			if (!other->m_exists) { continue; }

			if (!m_cdata.hasmorecontacts()) { return; }
            collision_detector::boxandbox(*block_, *other, &m_cdata);
        }
    }

    // check for sphere ground collisions
    if (m_ball_active) {
		if (!m_cdata.hasmorecontacts()) { return; }
        collision_detector::sphereandhalfspace(m_ball, plane, &m_cdata);
    }

}

void fracture::reset() {

    // only the first block exists
    m_blocks[0].m_exists = true;
    for (block *block_ = m_blocks+1; block_ < m_blocks+MAX_BLOCKS; block_++){
        block_->m_exists = false;
    }

    // set the first block
    m_blocks[0].m_half_size = _vec3(4,4,4);
    m_blocks[0].m_body->setposition(0, 7, 0);
    m_blocks[0].m_body->setorientation(1,0,0,0);
    m_blocks[0].m_body->setvelocity(0,0,0);
    m_blocks[0].m_body->setrotation(0,0,0);
    m_blocks[0].m_body->setmass(100.0f);

	_mat3 it;
    it.setblockinertiatensor(m_blocks[0].m_half_size, 100.0f);
    m_blocks[0].m_body->setinertiatensor(it);
    m_blocks[0].m_body->setdamping(0.9f, 0.9f);
    m_blocks[0].m_body->calculatederiveddata();
    m_blocks[0].calculateinternals();

    m_blocks[0].m_body->setacceleration( _utility::gravity );
    m_blocks[0].m_body->setawake(true);
    m_blocks[0].m_body->setcansleep(true);


    m_ball_active = true;

    // set up the ball
    m_ball.m_body->setposition(0,5.0f,20.0f);
    m_ball.m_body->setorientation(1,0,0,0);
    m_ball.m_body->setvelocity(
		m_random.randombinomial(4.0f),
        m_random.randomreal(1.0f, 6.0f),
        -20.0f
        );
    m_ball.m_body->setrotation(0,0,0);
    m_ball.m_body->calculatederiveddata();
    m_ball.m_body->setawake(true);
    m_ball.calculateinternals();

    m_hit = false;

    // reset the contacts
    m_cdata.m_contact_count = 0;
}

void fracture::updateobjects( float duration) {
    for (block *block = m_blocks; block < m_blocks+MAX_BLOCKS; block++) {
        if (block->m_exists){
            block->m_body->integrate(duration);
            block->calculateinternals();
        }
    }

    if (m_ball_active) {
        m_ball.m_body->integrate(duration);
        m_ball.calculateinternals();
    }
}

bool fracture::update(){

	// find the duration of the last frame in seconds
	float duration = application_clock->m_last_frame_seconds;
	if (duration > 0.0f) {

		if (duration > 0.05f) { duration = 0.05f; }

		// exit immediately if we aren't running the simulation
		if ( m_pause_simulation ) { return render(); }
		else if (m_auto_pause_simulation) {
			m_pause_simulation = true;
			m_auto_pause_simulation = false;
		}

		// update the objects
		updateobjects(duration);

		// perform the contact generation
		generatecontacts();

		// resolve detected contacts
		m_resolver.resolvecontacts(
			m_cdata.m_contact_array,
			m_cdata.m_contact_count,
			duration
			);

	}

	return render();
}

void fracture::clear(){
	application_releasecom(m_buffer);
	application_releasecom(m_point_buffer);
}

bool fracture::render(){

	if (m_hit) {
		m_blocks[0].divideblock(
			m_cdata.m_contact_array[m_fracture_contact],
			m_blocks,
			m_blocks+1
			);
		m_ball_active = false;
	}

	_mat4 mat = _application->m_view*_application->m_projection;

	// shader: flat ( lines and points ) technique
	application_throw_hr(_fx->SetTechnique(_api_manager->m_hflat_tech) );

	application_throw_hr(_fx->SetMatrix(_api_manager->m_hmvp,(D3DXMATRIX*)&mat ));

	application_throw_hr(_api_manager->m_d3ddevice->SetVertexDeclaration(_api_manager->m_flat_declaration));

	application_throw_hr(_api_manager->m_d3ddevice->SetStreamSource(0, m_buffer, 0, sizeof(_vec3)));

	/* draw ground ****************************************************************************/
	_material material;
	material.diffuse = _vec4(0.02f,0.02f,0.02f,1);
	application_throw_hr(_fx->SetValue(_api_manager->m_hmaterial, &(material), sizeof(_material)));

	_vec3 * v = 0;
	application_throw_hr( m_buffer->Lock(0, 0, (void**)&v, 0));
	v[0]  = _vec3(-20,0,0);
	v[1]  = _vec3(20,0,0);
	v[2]  = _vec3(0,0,-20);
	v[3]  = _vec3(0,0,20);
	application_throw_hr( m_buffer->Unlock()); 
	application_throw_hr(_fx->Begin(NULL, 0));
	application_throw_hr(_fx->BeginPass(0));
	application_throw_hr(_api_manager->m_d3ddevice->DrawPrimitive(D3DPT_LINELIST, 0, 2));
	application_throw_hr(_fx->EndPass());
	application_throw_hr(_fx->End()); 
	/******************************************************************************************/

	/* draw circles **************************************************************************/
	application_throw_hr(_api_manager->m_d3ddevice->SetStreamSource(0,m_point_buffer, 0, sizeof(_vec3)));
	application_throw_hr(_fx->Begin(NULL, 0));
	application_throw_hr(_fx->BeginPass(0));
	application_throw_hr(_api_manager->m_d3ddevice->DrawPrimitive(D3DPT_POINTLIST, 0, 3600));
	application_throw_hr(_fx->EndPass());
	application_throw_hr(_fx->End()); 
	/******************************************************************************************/


	/* shader: object technique */
	application_throw_hr( _fx->SetTechnique(_api_manager->m_htech) );


	/* draw blocks *****************************************************************************/
	for (block *block_ = m_blocks; block_ < m_blocks+MAX_BLOCKS; block_++) {
		if (block_->m_exists) {
			_vec3 scale = _vec3(block_->m_half_size.x*2, block_->m_half_size.y*2, block_->m_half_size.z*2);
			_mat4 world =  _scale(scale) * block_->gettransform();

			if (block_->m_body->getawake()) { material.diffuse = _vec4(1.0f,0.7f,0.7f,1.0f ); }
			else { material.diffuse = _vec4(0.7f,0.7f,1.0f,1.0f); }
			_application->m_object_manager->drawcube( world , material.diffuse );
		}
	}
	/*******************************************************************************************/

	/* draw ball *******************************************************************************/
	if ( m_ball_active ) {
		_mat4 world =  _scale(_vec3(0.25f, 0.25f , 0.25f )) * _translate( m_ball.m_body->getposition() );

		material.diffuse = _vec4(0.4f, 0.7f, 0.4f,1.0f );
		_application->m_object_manager->drawsphere( world , material.diffuse );
	}
	/*******************************************************************************************/

	/* draw contacts if debug is toggled*/
	return drawdebug(); 
}

bool fracture::drawdebug() {

	if (!m_render_debug_info) { return true; }

	/* disable depth buffer*/
	application_throw_hr( _api_manager->m_d3ddevice->SetRenderState(D3DRS_ZENABLE,D3DZB_FALSE) );

	// shader: flat (lines) technique
	application_error_hr(_fx->SetTechnique(_api_manager->m_hflat_tech) );
	_mat4 mat = _application->m_view*_application->m_projection;
	application_throw_hr(_fx->SetMatrix(_api_manager->m_hmvp,(D3DXMATRIX*)&mat ));

	application_throw_hr(_api_manager->m_d3ddevice->SetVertexDeclaration(_api_manager->m_flat_declaration));

	application_throw_hr(_api_manager->m_d3ddevice->SetStreamSource(0,m_buffer, 0, sizeof(_vec3)));

    // recalculate the contacts, so they are current (in case we're
    // paused, for example).
    generatecontacts();

    // render the contacts, if required
	_vec3 * v =0;
	uint32_t pos =0;
	for (uint32_t i = 0; i < m_cdata.m_contact_count; i++) {
        // interbody contacts are in green, floor contacts are red.
		_material material;
        if (m_contacts[i].m_body[1]) {
			material.diffuse = _vec4(0.0f,1.0f,0.0f,1.0f);
        } else {
			material.diffuse = _vec4(1.0f,0.0f,0.0f,1.0f);
        }
		application_throw_hr(_fx->SetValue(_api_manager->m_hmaterial, &(material), sizeof(_material)));
		application_throw_hr(m_buffer->Lock(0, 0, (void**)&v, 0));
		v[0] = m_contacts[i].m_contact_point;
		v[1] = m_contacts[i].m_contact_point+m_contacts[i].m_contact_normal;
		application_throw_hr(m_buffer->Unlock()); 
		application_throw_hr(_fx->Begin(NULL, 0));
		application_throw_hr(_fx->BeginPass(0));
		application_throw_hr(_api_manager->m_d3ddevice->DrawPrimitive(D3DPT_LINELIST, 0, 1));
		application_throw_hr(_fx->EndPass());
		application_throw_hr(_fx->End()); 

	}
	/* enable depth buffer*/
	application_throw_hr( _api_manager->m_d3ddevice->SetRenderState(D3DRS_ZENABLE,D3DZB_TRUE) );
	return true;
}
void fracture::key(uint8_t key) {

	switch(key)
	{
	case 'R': case 'r':
		// Reset the simulation
		reset();
		return;

	case 'C': case 'c':
		// Toggle rendering of contacts
		m_render_debug_info = !m_render_debug_info;
		return;

	case 'P': case 'p':
		// Toggle running the simulation
		m_pause_simulation = !m_pause_simulation;
		return;

	case ' ':
		// Advance one frame
		m_auto_pause_simulation = true;
		m_pause_simulation = false;
	}

}