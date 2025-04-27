#include "explosion.h"


#include "clock.h"
#include "application.h"
#include "d3d_manager.h"
#include "object_manager.h"



explosion::explosion(): 
m_resolver(maxcontacts*8),
	m_render_debug_info(false),
	m_pause_simulation(true),
	m_auto_pause_simulation(false),
	m_edit_mode(false),
	m_up_mode(false)
{

}


bool explosion::init(){

	m_cdata.m_contact_array = m_contacts;

	m_state_string = _string("reset: R run simulation: P render contacts: C advance frame: spcae fire: Q");

	/* point buffer ***************************************************/
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
	/******************************************************************/

	/* line buffer ****************************************************/
	m_buffer = NULL;
	application_throw_hr(_api_manager->m_d3ddevice->CreateVertexBuffer(
		4* sizeof(_vec3),
		D3DUSAGE_WRITEONLY,0, D3DPOOL_MANAGED,	
		&m_buffer, 0));
	if(!m_buffer){ application_throw("line buffer"); }
	/******************************************************************/

	_application->addflags(application_spherical_cam);

	//camera
	_application->m_distance = 40.0f;
	_application->m_pitch        = -35.0f;
	_application->m_pitch_pos    = -35.0f;

	// set up the initial block
	reset();

	return true;
}

void explosion::clear(){
	application_releasecom(m_buffer);
	application_releasecom(m_point_buffer);
}

void explosion::fire() {
	_vec3 pos = m_ball_data[0].m_body->getposition();
	pos.normalise();

	m_ball_data[0].m_body->addforce(pos * -1000.0f);
}

void explosion::generatecontacts() {


	// note that this method makes a lot of use of early returns to avoid
	// processing lots of potential contacts that it hasn't got room to
	// store.

	// create the ground plane data
	collision_plane plane;
	plane.m_direction = _vec3(0,1,0);
	plane.m_offset = 0;

	// set up the collision data structure
	m_cdata.reset(maxcontacts);
	m_cdata.m_friction    = 0.9f;
	m_cdata.m_restitution = 0.6f;
	m_cdata.m_tolerance   = 0.1f;

	// perform exhaustive collision detection
	_mat4 transform, othertransform;
	_vec3 position, otherposition;
	for (box *box_ = m_box_data; box_ < m_box_data+m_boxes; box_++) {
		// check for collisions with the ground plane
		if (!m_cdata.hasmorecontacts()) { return; }
		collision_detector::boxandhalfspace(*box_, plane, &m_cdata);

		// check for collisions with each other box
		for (box *other = box_+1; other < m_box_data+m_boxes; other++){
			if (!m_cdata.hasmorecontacts()) { return; }

			collision_detector::boxandbox(*box_, *other, &m_cdata);

			if (intersection_tests::boxandbox(*box_, *other)){
				box_->m_is_over_lapping = other->m_is_over_lapping = true;
			}
		}

		// check for collisions with each ball
		for (ball *other = m_ball_data; other < m_ball_data+m_balls; other++){
			if (!m_cdata.hasmorecontacts()) { return; }
			collision_detector::boxandsphere(*box_, *other, &m_cdata);
		}
	}

	for (ball *ball_ = m_ball_data; ball_ < m_ball_data+m_balls; ball_++) {
		// check for collisions with the ground plane
		if (!m_cdata.hasmorecontacts()) { return; }
		collision_detector::sphereandhalfspace(*ball_, plane, &m_cdata);

		for (ball *other = ball_+1; other < m_ball_data+m_balls; other++) {
			// check for collisions with the ground plane
			if (!m_cdata.hasmorecontacts()) { return; }
			collision_detector::sphereandsphere(*ball_, *other, &m_cdata);
		}
	}

}

void explosion::reset() {

	box *box_ = m_box_data;

	box_->setstate(
		_vec3(0,3,0),
		_quaternion(),
		_vec3(4,1,1),
		_vec3(0,1,0)
		);
	box_++;

	if ( m_boxes > 1) {

		box_->setstate(
			_vec3(0.0f,4.75f,2.0f),
			_quaternion(1.0f,0.1f,0.05f,0.01f),
			_vec3(1.0f,1.0f,4.0f),
			_vec3(0.0f,1.0f,0.0f)
			);
	}
	box_++;
	// create the random objects
	int p = 0;
	random random;
	for (; box_ < m_box_data+m_boxes; box_++) { 
		box_->random(&random);
	}

	for (ball *ball_ = m_ball_data; ball_ < m_ball_data+m_balls; ball_++){
		ball_->random(&random);
	}
	// reset the contacts
	m_cdata.m_contact_count = 0;

}

void explosion::updateobjects( float duration) {
	// update the physics of each box in turn

	static int update =0;

	for (box *box_ = m_box_data; box_ < m_box_data+m_boxes; box_++) {
		// run the physics
		box_->m_body->integrate(duration);
		box_->calculateinternals();
		box_->m_is_over_lapping = false;
	}

	// update the physics of each ball in turn
	for (ball *ball_ = m_ball_data; ball_ < m_ball_data+m_balls; ball_++) {
		// run the physics
		ball_->m_body->integrate(duration);
		ball_->calculateinternals();
	}

}

bool explosion::update(){

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

bool explosion::render(){

	// enable alpha blending.
	application_error_hr(_api_manager->m_d3ddevice->SetRenderState(D3DRS_ALPHABLENDENABLE, true));
	application_error_hr(_api_manager->m_d3ddevice->SetRenderState(D3DRS_SRCBLEND, D3DBLEND_SRCALPHA));
	application_error_hr(_api_manager->m_d3ddevice->SetRenderState(D3DRS_DESTBLEND, D3DBLEND_INVSRCALPHA));

	// update the transform matrices of each box in turn
	for (box *box_ = m_box_data; box_ < m_box_data+m_boxes; box_++) {
		box_->calculateinternals();
		box_->m_is_over_lapping = false;
	}

	// update the transform matrices of each ball in turn
	for (ball *ball_ = m_ball_data; ball_ < m_ball_data+m_balls; ball_++) {
		// run the physics
		ball_->calculateinternals();
	}


	/* shader: flat (ground points and lines) */
	application_error_hr(_fx->SetTechnique(_api_manager->m_hflat_tech) );

	_mat4 mat = _application->m_view*_application->m_projection;

	application_throw_hr(_fx->SetMatrix(_api_manager->m_hmvp,(D3DXMATRIX*)&mat ));

	application_throw_hr(_api_manager->m_d3ddevice->SetVertexDeclaration(_api_manager->m_flat_declaration));

	application_throw_hr(_api_manager->m_d3ddevice->SetStreamSource(0,m_buffer, 0, sizeof(_vec3)));

	/* draw ground ****************************************************************************/
	_material material;
	material.diffuse = _vec4(0.02f,0.02f,0.02f,1);
	application_throw_hr(_fx->SetValue(_api_manager->m_hmaterial, &(material), sizeof(_material)));
	_vec3 * v = 0;
	application_throw_hr(m_buffer->Lock(0, 0, (void**)&v, 0));
	v[0]  = _vec3(-20,0,0);
	v[1]  = _vec3(20,0,0);
	v[2]  = _vec3(0,0,-20);
	v[3]  = _vec3(0,0,20);
	application_throw_hr(m_buffer->Unlock()); 

	/* draw lines*/
	application_throw_hr(_fx->Begin(NULL, 0));
	application_throw_hr(_fx->BeginPass(0));
	application_throw_hr(_api_manager->m_d3ddevice->DrawPrimitive(D3DPT_LINELIST, 0, 2));
	application_throw_hr(_fx->EndPass());
	application_throw_hr(_fx->End()); 


	/* draw circles*/
	application_throw_hr(_api_manager->m_d3ddevice->SetStreamSource(0,m_point_buffer, 0, sizeof(_vec3)));
	application_throw_hr(_fx->Begin(NULL, 0));
	application_throw_hr(_fx->BeginPass(0));
	application_throw_hr(_api_manager->m_d3ddevice->DrawPrimitive(D3DPT_POINTLIST, 0, 3600));
	application_throw_hr(_fx->EndPass());
	application_throw_hr(_fx->End()); 
	/******************************************************************************************/


	application_error_hr(_fx->SetTechnique(_api_manager->m_htech) );

	_mat4 mirror;
	mirror[1][1]=-1;
	/* draw blocks *****************************************************************************/
	for (box *box_ = m_box_data; box_ < m_box_data+m_boxes; box_++) {

		/* box *****************************************************************************/
		_vec3 scale = _vec3(box_->m_half_size.x*2, box_->m_half_size.y*2, box_->m_half_size.z*2);
		_mat4 world =  _scale(scale) * box_->gettransform();
		if ( box_->m_body->getawake()) { material.diffuse = _vec4(1.0f,0.7f,0.7f,1.0f ); }
		else { material.diffuse = _vec4(0.7f,0.7f,1.0f,1.0f); }
		_application->m_object_manager->drawcube( world , material.diffuse );
		/***********************************************************************************/
		/* mirror **************************************************************************/
		_application->m_object_manager->drawcube( world*mirror, material.diffuse ,true);
		/***********************************************************************************/
		/* shadow **************************************************************************/
		world =  _scale(scale)*box_->gettransform() *_scale(_vec3(1.0f,0.0f,1.0f)) ;
		_application->m_object_manager->drawcube( world , _vec4(0.0f,0.0f,0.0f,0.3f) ,false);
		/***********************************************************************************/
	}
	/*******************************************************************************************/

	/* draw ball *******************************************************************************/
	for (ball *ball_ = m_ball_data; ball_ < m_ball_data+m_balls; ball_++) {

		/* ball ****************************************************************************/
		_vec3 scale = _vec3(ball_->m_radius*2, ball_->m_radius*2, ball_->m_radius*2);
		_mat4 world =  _scale( scale ) * _translate( ball_->m_body->getposition() );
		if ( ball_->m_body->getawake()) { material.diffuse = _vec4(1.0f,0.7f,0.7f,1.0f ); }
		else { material.diffuse = _vec4(0.7f,0.7f,1.0f,1.0f); }
		_application->m_object_manager->drawsphere( world , material.diffuse );
		/***********************************************************************************/
		/* mirror **************************************************************************/
		_application->m_object_manager->drawsphere( world*mirror , material.diffuse ,true);
		/***********************************************************************************/
		/* shadow **************************************************************************/
		world =  _scale(scale)*ball_->gettransform() *_scale(_vec3(1.0f,0.0f,1.0f)) ;
		_application->m_object_manager->drawsphere( world , _vec4(0.0f,0.0f,0.0f,0.3f) ,false);
		/***********************************************************************************/
	}
	/*******************************************************************************************/

	// disable alpha blending.
	application_error_hr(_api_manager->m_d3ddevice->SetRenderState(D3DRS_ALPHABLENDENABLE, false));

	/* draw contacts if debug is toggled*/
	return drawdebug(); 
}

bool explosion::drawdebug() {

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



void explosion::key(uint8_t key) {

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
		return;
	case'Q': case 'q':
		fire();
		return;
	}

}
