#include "ballistic.h"

#include "clock.h"
#include "application.h"
#include "d3d_manager.h"
#include "object_manager.h"

static const char * shot_type_string [] = { "PISTOL", "ARTILLERY", "FIREBALL", "LASER" };

bool ballistic::init() {

	/* reference line buffer **********************************************/
	m_buffer = NULL;
	application_throw_hr(_api_manager->m_d3ddevice->CreateVertexBuffer(
		70 * sizeof(_vec3),
		D3DUSAGE_WRITEONLY,0, D3DPOOL_MANAGED,	
		&m_buffer, 0));
	if(!m_buffer){ application_throw("line buffer"); }

	_vec3 * v = 0;
	application_throw_hr( m_buffer->Lock(0, 0, (void**)&v, 0));
	for(uint32_t space = 0, ii=0; ii<70; ){
		v[ii].x   = -5.0f;
		v[ii].z = float(space);

		ii++; 
		space+=2;

		v[ii].x   = 5.0f;
		v[ii++].z = float(space);
	}
	application_throw_hr( m_buffer->Unlock());
	/**********************************************************************/

	// set default shottype
	m_current_shot_type = LASER;

   //set state string
	m_state_string = _string("fire: left mouse button ");
	m_state_string = m_state_string + _string(" current shot type : ");
	m_state_string = m_state_string + shot_type_string[m_current_shot_type-1];


	// make all shots unused
	for (ammo_round *shot = m_ammo; shot < m_ammo+m_ammo_rounds; shot++) { shot->m_type = UNUSED; }

	return true;
}

void ballistic::clear() {
	application_releasecom(m_buffer);
}

bool ballistic::update() {

	// duration of the last frame in seconds
	float duration = application_clock->m_last_frame_seconds;
	if (duration > 0.0f){

		// update the physics of each particle in turn
		for (ammo_round *shot = m_ammo; shot < m_ammo+m_ammo_rounds; shot++){

			if (shot->m_type != UNUSED){
				// run the physics
				shot->m_particle.integrate(duration);

				shot->m_update_time += application_clock->m_last_frame_seconds;
				// check if the particle is now invalid
				if (shot->m_particle.getposition().y < 0.0f ||
					shot->m_update_time> 5.0f /*seconds*/ ||
					shot->m_particle.getposition().z > 200.0f)
				{
					// we simply set the shot type to be unused, so the
					// memory it occupies can be reused by another shot.
					shot->m_type = UNUSED;
				}
			}
		}
	}
	return render();
}

bool ballistic::render() {

	//set camera view matrix
	_application->m_view = _lookatrh(_vec3(-16.0f, 2.0f, -1.5f),_vec3(0.0f, 5.0f, 12.0f), _utility::up);

	// shader: flat (lines) technique
	application_error_hr(_fx->SetTechnique(_api_manager->m_hflat_tech) );

	/* draw reference lines ************************************************************/
	_material material;
	material.diffuse   = _vec4(0.0f,0.0f,0.0f,1.0f);
	_mat4 mat = _application->m_view*_application->m_projection;
	application_throw_hr(_fx->SetMatrix(_api_manager->m_hmvp,(D3DXMATRIX*)&mat ));

	application_throw_hr(_api_manager->m_d3ddevice->SetVertexDeclaration(_api_manager->m_flat_declaration));

	application_throw_hr(_api_manager->m_d3ddevice->SetStreamSource(0,m_buffer, 0, sizeof(_vec3)));

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
	mat = _translate( _vec3(0.0f,4.0f, 0.0f) );
	mat = mat * _scale(_vec3(0.1f,0.1f,0.1f));
	if( !_application->m_object_manager->drawsphere( mat ,_vec4(0.0f,0.0f,0.0f,1.0f)) ){ return false; }
	/*shadow***********************************************************/
	mat = _translate( _vec3(0.0f, 4.0f, 0.0f) );
	mat = mat * _scale( _vec3(0.1f,0.01f,0.1f ));
	if( !_application->m_object_manager->drawsphere( mat ,_vec4(0.0f,0.0f,0.0f,1.0f)) ){ return false; }
	/******************************************************************/


	/* draw ammo particles */
	for (ammo_round *shot = m_ammo; shot < m_ammo+m_ammo_rounds; shot++) {
		if (shot->m_type != ballistic::UNUSED) {

			/* particle size */
			float scale = 0.0f;
			/******************************************************************/

			/*1:pistol(gray) 2:artillery(blue) 3:fireball(red) 4:laser(green) */
			_vec4 color;
			/******************************************************************/

			switch(shot->m_type){
			case 1:scale = 0.2f;  color = _vec4(0.4f,0.4f,0.4f,1.0f); break;
			case 2:scale = 0.6f;  color = _vec4(0.0f,0.0f,1.0f,1.0f); break;
			case 3:scale = 0.45f; color = _vec4(1.0f,0.0f,0.0f,1.0f); break;
			case 4:scale = 0.4f;  color = _vec4(0.0f,1.0f,0.0f,1.0f); break;
			}

			/* set position */
			mat = _translate( shot->m_particle.m_position );

			/*round************************************************************/
			mat = mat * _scale(_vec3(scale,scale,scale));
			if( !_application->m_object_manager->drawsphere( mat ,color) ){ return false; }
			/*shadow***********************************************************/
			mat = _translate( shot->m_particle.m_position );
			mat = mat * _scale(_vec3(scale,0.01f,scale));
			if( !_application->m_object_manager->drawsphere( mat ,_vec4(0.0f,0.0f,0.0f,1.0f)) ){ return false; }
			/******************************************************************/
		}
	}
	return true;
}

void ballistic::leftmouse() { /* fire */

	// find the first available round
	ammo_round *shot;
	for (shot = m_ammo; shot < m_ammo+m_ammo_rounds; shot++) {
		if (shot->m_type == UNUSED) { break; }
	}

	// if we didn't find a round, then exit - we can't fire.
	if (shot >= m_ammo+m_ammo_rounds) { return; }

	// set the properties of the particle
	switch(m_current_shot_type)
	{
	case PISTOL:
		shot->m_particle.setmass(2.0f); // 2.0kg
		shot->m_particle.setvelocity(0.0f, 0.0f, 35.0f); // 35m/s
		shot->m_particle.setacceleration(0.0f, -1.0f, 0.0f);
		shot->m_particle.setdamping(0.99f);
		break;

	case ARTILLERY:
		shot->m_particle.setmass(200.0f); // 200.0kg
		shot->m_particle.setvelocity(0.0f, 30.0f, 40.0f); // 50m/s
		shot->m_particle.setacceleration(0.0f, -20.0f, 0.0f);
		shot->m_particle.setdamping(0.99f);
		break;

	case FIREBALL:
		shot->m_particle.setmass(1.0f); // 1.0kg - mostly blast damage
		shot->m_particle.setvelocity(0.0f, 0.0f, 10.0f); // 5m/s
		shot->m_particle.setacceleration(0.0f, 0.6f, 0.0f); // Floats up
		shot->m_particle.setdamping(0.9f);
		break;

	case LASER:
		// note that this is the kind of laser bolt seen in films,
		// not a realistic laser beam!
		shot->m_particle.setmass(0.1f); // 0.1kg - almost no weight
		shot->m_particle.setvelocity(0.0f, 0.0f, 100.0f); // 100m/s
		shot->m_particle.setacceleration(0.0f, 0.0f, 0.0f); // No gravity
		shot->m_particle.setdamping(0.99f);
		break;
	}

	// set the data common to all particle types
	shot->m_particle.setposition(0.0f, 1.5f, 0.0f);
	shot->m_update_time = 0;/* count in seconds*/
	shot->m_type = m_current_shot_type;

	// clear the force accumulators
	shot->m_particle.clearaccumulator();
}

void ballistic::key( uint8_t key) {

	switch(key)
	{
	case '1': m_current_shot_type = PISTOL;    break;
	case '2': m_current_shot_type = ARTILLERY; break;
	case '3': m_current_shot_type = FIREBALL;  break;
	case '4': m_current_shot_type = LASER;     break;
	}

	m_state_string = _string("fire: left mouse button ");
	m_state_string = m_state_string + _string(" current shot type : ");
	m_state_string = m_state_string + shot_type_string[m_current_shot_type-1];
}