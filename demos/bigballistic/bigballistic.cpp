#include "bigballistic.h"


#include "clock.h"
#include "application.h"
#include "d3d_manager.h"
#include "object_manager.h"


static const char * shot_type_string [] = { "PISTOL", "ARTILLERY", "FIREBALL", "LASER" };

bigballistic::bigballistic(): 
m_resolver(maxcontacts*8),
	m_pause_simulation(false),
	m_auto_pause_simulation(false),
	m_current_shot_type(LASER)
{

}


bool bigballistic::init(){

	m_cdata.m_contact_array = m_contacts;

	m_buffer = NULL;

	/* reference line buffer **********************************************/
	application_throw_hr(_api_manager->m_d3ddevice->CreateVertexBuffer(
		70 * sizeof(_vec3),
		D3DUSAGE_WRITEONLY,0, D3DPOOL_MANAGED,	
		&m_buffer, 0));
	if(!m_buffer){ application_throw("line buffer"); }

	_vec3 * v = 0;
	application_throw_hr(m_buffer->Lock(0, 0, (void**)&v, 0));
	for(uint32_t space = 0, ii=0; ii<70; ){
		v[ii].x   = -5.0f;
		v[ii].z = float(space);

		ii++; 
		space+=2;

		v[ii].x   = 5.0f;
		v[ii++].z = float(space);
	}
	application_throw_hr(m_buffer->Unlock());
	/**********************************************************************/

	// set up the initial block
	reset();

	// set default shottype
	m_current_shot_type = LASER;

	//set state string
	m_state_string = _string("LeftMouse: fire | select ammo type(1:pistol|2:artillery|3:fireball|4:laser)");
	m_state_string = m_state_string + _string(" current :");
	m_state_string = m_state_string + shot_type_string[m_current_shot_type-1];

	return true;
}

void bigballistic::generatecontacts() {

	// create the ground plane data
	collision_plane plane;
	plane.m_direction = _vec3(0,1,0);
	plane.m_offset = 0;

	// set up the collision data structure
	m_cdata.reset(maxcontacts);
	m_cdata.m_friction    = 0.9f;
	m_cdata.m_restitution = 0.1f;
	m_cdata.m_tolerance   = 0.1f;

	// check ground plane collisions
	for (box *box = m_box_data; box < m_box_data+m_boxes; box++) {

		if (!m_cdata.hasmorecontacts()) { return; }

		collision_detector::boxandhalfspace(*box, plane, &m_cdata);


		// check for collisions with each shot
		for (ammo_round *shot = m_ammo; shot < m_ammo+m_ammo_rounds; shot++) {
			if (shot->m_type != UNUSED){

				if (!m_cdata.hasmorecontacts()) { return; }

				// when we get a collision, remove the shot
				if ( collision_detector::boxandsphere(*box, *shot, &m_cdata) ){
					shot->m_type = UNUSED;
				}
			}
		}
	}

}

void bigballistic::reset() {

	// make all shots unused
	for (ammo_round *shot = m_ammo; shot < m_ammo+m_ammo_rounds; shot++) { shot->m_type = UNUSED; }

	// initialise the box
	float z = 20.0f;
	for (box *box = m_box_data; box < m_box_data+m_boxes; box++) {
		box->setstate(z);
		z += 90.0f;
	}

}

void bigballistic::updateobjects( float duration) {

	// update the physics of each particle in turn
	for (ammo_round *shot = m_ammo; shot < m_ammo+m_ammo_rounds; shot++) {
		if (shot->m_type != UNUSED) {
			// run the physics
			shot->m_body->integrate(duration);
			shot->calculateinternals();

			shot->m_update_time += application_clock->m_last_frame_seconds;

			// check if the particle is now invalid
			if (shot->m_body->getposition().y < 0.0f ||
				shot->m_update_time>5.0f /*seconds*/ ||
				shot->m_body->getposition().z > 200.0f)
			{
				// we simply set the shot type to be unused, so the
				// memory it occupies can be reused by another shot.
				shot->m_type = UNUSED;
			}
		}
	}

	// update the boxes
	for (box *box = m_box_data; box < m_box_data+m_boxes; box++) {
		// run the physics
		box->m_body->integrate(duration);
		box->calculateinternals();
	}

}

bool bigballistic::update(){

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

void bigballistic::clear(){
	application_releasecom(m_buffer);
}

bool bigballistic::render(){

	//set camera view matrix
	_application->m_view =_lookatrh(_vec3(-60.0f,10.0f, -1.5f),_vec3(0.0f, 20.0f, 40.0f), _utility::up);

	// shader: flat (lines) technique
	application_error_hr(_fx->SetTechnique(_api_manager->m_hflat_tech) );

	/* draw reference lines ************************************************************/
	_material material;
	material.diffuse = _vec4(0.02f,0.02f,0.02f,1);

	_mat4 mat = _application->m_view*_application->m_projection;

	application_throw_hr(_fx->SetMatrix(_api_manager->m_hmvp,(D3DXMATRIX*)&mat ));

	application_throw_hr(_api_manager->m_d3ddevice->SetVertexDeclaration(_api_manager->m_flat_declaration));

	application_throw_hr(_api_manager->m_d3ddevice->SetStreamSource(0, m_buffer, 0, sizeof(_vec3)));

	application_throw_hr(_fx->SetValue(_api_manager->m_hmaterial, &(material), sizeof(_material)));

	application_throw_hr(_fx->Begin(NULL, 0));
	application_throw_hr(_fx->BeginPass(0));
	application_throw_hr(_api_manager->m_d3ddevice->DrawPrimitive(D3DPT_LINELIST, 0, 35 ));
	application_throw_hr(_fx->EndPass());
	application_throw_hr(_fx->End()); 
	/***********************************************************************************/

	/* shader: object technique */
	application_error_hr(_fx->SetTechnique(_api_manager->m_htech) );

	// draw a sphere at the firing point, and add a shadow projected
	// onto the ground plane.
	/*sphere***********************************************************/
	mat = _translate( _vec3(0.0f,2.0f, 0.0f) );
	mat = mat * _scale(_vec3(0.3f,0.3f,0.3f));
	if( !_application->m_object_manager->drawsphere( mat ,_vec4(0.0f,0.0f,0.0f,1.0f)) ){ return false; }
	/*shadow***********************************************************/
	mat = _translate( _vec3(0.0f, 2.0f, 0.0f) );
	mat = mat * _scale( _vec3(0.3f,0.01f,0.3f ));
	if( !_application->m_object_manager->drawsphere( mat ,_vec4(0.0f,0.0f,0.0f,1.0f)) ){ return false; }
	/******************************************************************/

	/* draw boxes */
	for (box *box_ = m_box_data; box_ < m_box_data+m_boxes; box_++){

		_vec3 scale = _vec3(box_->m_half_size.x*2, box_->m_half_size.y*2, box_->m_half_size.z*2);
		_mat4 world =  _scale(scale) * box_->gettransform();

		if ( box_->m_body->getawake()) { material.diffuse = _vec4(0.8f,0.4f,0.4f,1.0f ); }
		else { material.diffuse = _vec4(0.7f,0.7f,1.0f,1.0f); }
		_application->m_object_manager->drawcube( world , material.diffuse );
	}

	/* draw ammo particles */
	for (ammo_round *shot = m_ammo; shot < m_ammo+m_ammo_rounds; shot++) {
		if (shot->m_type != UNUSED) {

			/*round************************************************************/
			_vec3 scale = _vec3(shot->m_radius*2, shot->m_radius*2, shot->m_radius*2);
			_mat4 world =  _scale( scale ) * _translate( shot->m_body->getposition() );
			_application->m_object_manager->drawsphere( world , _vec4(1.0f,0.0f,0.0f,1.0f ) );
			/*shadow***********************************************************/
			scale = _vec3(shot->m_radius*2, .01f, shot->m_radius*2);
			world =  _scale( scale ) * _translate( shot->m_body->getposition() );
			_application->m_object_manager->drawsphere( world , _vec4(0.0f,0.0f,0.0f,1.0f ) );
			/******************************************************************/
		}
	}

	return true;
}

void bigballistic::leftmouse(){
	// find the first available round.
	ammo_round *shot;
	for (shot = m_ammo; shot < m_ammo+m_ammo_rounds; shot++) {
		if (shot->m_type == UNUSED) { break; }
	}

	// if we didn't find a round, then exit - we can't fire.
	if (shot >= m_ammo+m_ammo_rounds) { return; }

	// set the shot
	shot->setstate(m_current_shot_type);
}

void bigballistic::key(uint8_t key) {

	switch(key)
	{
	case '1': m_current_shot_type = PISTOL; break;
	case '2': m_current_shot_type = ARTILLERY; break;
	case '3': m_current_shot_type = FIREBALL; break;
	case '4': m_current_shot_type = LASER; break;

	case 'r': case 'R': reset(); break;
	}

	m_state_string = _string("LeftMouse: fire | select ammo type(1:pistol|2:artillery|3:fireball|4:laser)");
	m_state_string = m_state_string + _string(" current shot type :");
	m_state_string = m_state_string + shot_type_string[m_current_shot_type-1];

}
