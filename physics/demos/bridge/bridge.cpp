#include "bridge.h"


#include "clock.h"
#include "d3d_manager.h"

#include "object_manager.h"

#include "application.h"

bridge::bridge() : m_world(120) {
	m_cables = NULL;
	m_rods = NULL;
	m_supports = NULL;
	m_particle_array = NULL;
}

bool bridge::init(){

	m_state_string = _string("A:left|D:right|W:forward|S:back");

	/* line buffer ******************************************************/
	m_buffer = NULL;
	application_throw_hr(_api_manager->m_d3ddevice->CreateVertexBuffer(
		(support_count*2) * sizeof(_vec3),
		D3DUSAGE_WRITEONLY,0, D3DPOOL_MANAGED,	
		&m_buffer, 0));
	if(!m_buffer){ application_throw("line buffer"); }
	/********************************************************************/


	m_mass_pos = _vec3(0.0f,0.0f,0.5f);

	/* allocate particles*/
	m_particle_array = new particle[12];
	for (uint32_t i = 0; i < 12; i++){
		m_world.m_particles.pushback( m_particle_array + i);
	}

	/* init ground contact generator */
	m_ground_contact_generator.init( &m_world.m_particles );
	m_world.getcontactgenerators().pushback( &m_ground_contact_generator );

	// create the masses and connections
	for (uint32_t i = 0; i < 12; i++) {
		uint32_t x = (i%12)/2;
		m_particle_array[i].setposition( float(i/2)*2.0f-5.0f,  4, float(i%2)*2.0f-1.0f );
		m_particle_array[i].setvelocity(0,0,0);
		m_particle_array[i].setdamping(0.9f);
		m_particle_array[i].setacceleration(_utility::gravity);
		m_particle_array[i].clearaccumulator();
	}

	// add the links
	m_cables = new particle_cable[cable_count];
	for (uint32_t i = 0; i < 10; i++) {
		m_cables[i].m_contact_particles[0] = &m_particle_array[i];
		m_cables[i].m_contact_particles[1] = &m_particle_array[i+2];
		m_cables[i].m_max_length = 1.9f;
		m_cables[i].m_restitution = 0.3f;
		m_world.getcontactgenerators().pushback(&m_cables[i]);
	}

	m_supports = new particle_cable_constraint[support_count];
	for (uint32_t i = 0; i < support_count; i++) {
		m_supports[i].m_particle = m_particle_array+i;
		m_supports[i].m_anchor = _vec3( float(i/2)*2.2f-5.5f, 6, float(i%2)*1.6f-0.8f );

		if (i < 6) { m_supports[i].m_max_length = float(i/2)*0.5f + 3.0f; }
		else { m_supports[i].m_max_length = 5.5f - float(i/2)*0.5f; }

		m_supports[i].m_restitution = 0.5f;
		m_world.getcontactgenerators().pushback( &m_supports[i] );
	}

	m_rods = new particle_rod[rod_count];
	for (uint32_t i = 0; i < 6; i++) {
		m_rods[i].m_contact_particles[0] = &m_particle_array[i*2];
		m_rods[i].m_contact_particles[1] = &m_particle_array[i*2+1];
		m_rods[i].m_length = 2;
		m_world.getcontactgenerators().pushback(&m_rods[i]);
	}
	updateadditionalmass(); 


	return true;
}

void bridge::clear(){
	if (m_cables)         { delete[] m_cables; }
	if (m_rods)           { delete[] m_rods; }
	if (m_supports)       { delete[] m_supports; }
	if (m_particle_array) { delete [] m_particle_array; }
	application_releasecom(m_buffer);
}

bool bridge::update(){

	// clear accumulators
	m_world.startframe();

	// find the duration of the last frame in seconds
	float duration = application_clock->m_last_frame_seconds;
	if (duration > 0.0f) { 

		// run the simulation
		m_world.runphysics(duration);

		updateadditionalmass();
	}
	return render();
}

bool bridge::render(){

	//set camera view matrix
	_application->m_view = _lookatrh(_vec3(0.0f, 4.0f, 12.0f),_vec3(0.0f, 4.0f, 0.0f),_utility::up);

	// shader: flat ( rods, cables, supports) technique
	application_error_hr(_fx->SetTechnique(_api_manager->m_hflat_tech) );

	_vec3 * v = 0;
	/* 0-------0*/
	/* 0-------0*/
	/* 0-------0*/
	/* 0-------0 rod line buffer */
	application_throw_hr(!m_buffer->Lock(0, 0, (void**)&v, 0));
	for(uint32_t i=0,ii=0; ii<rod_count;   ii++ ){
		v[i++] = m_rods[ii].m_contact_particles[0]->getposition();
		v[i++] = m_rods[ii].m_contact_particles[1]->getposition();
	}
	application_throw_hr(!m_buffer->Unlock());


	_mat4 mat = _application->m_view*_application->m_projection;

	application_throw_hr(_fx->SetMatrix(_api_manager->m_hmvp,(D3DXMATRIX*)&mat ));

	application_throw_hr(_api_manager->m_d3ddevice->SetVertexDeclaration(_api_manager->m_flat_declaration));

	application_throw_hr(_api_manager->m_d3ddevice->SetStreamSource(0,m_buffer, 0, sizeof(_vec3)));
	
	/* draw rods ***********************************************************************************/
	_material material;
	material.diffuse   = _vec4(0.0f,0.0f,1.0f,1.0f);
	application_throw_hr(_fx->SetValue(_api_manager->m_hmaterial, &(material), sizeof(_material)));

	application_throw_hr(_fx->Begin(NULL, 0));
	application_throw_hr(_fx->BeginPass(0));
	application_throw_hr(_api_manager->m_d3ddevice->DrawPrimitive(D3DPT_LINELIST, 0, rod_count ));
	application_throw_hr(_fx->EndPass());
	application_throw_hr(_fx->End()); 
	/***********************************************************************************************/

	application_throw_hr(!m_buffer->Lock(0, 0, (void**)&v, 0));
	/* 0*/
	/* |*/
	/* 0*/
	/* |*/
	/* 0*/
	/* |*/
	/* 0 cable line buffer */
	for(uint32_t i=0,ii=0; ii<cable_count;   ii++ ){
		v[i++] = m_cables[ii].m_contact_particles[0]->getposition();
		v[i++] = m_cables[ii].m_contact_particles[1]->getposition();
	}
	application_throw_hr(!m_buffer->Unlock());

	/* draw cables *********************************************************************************/
	material.diffuse   = _vec4(0.0f,1.0f,0.0f,1.0f);
	application_throw_hr(_fx->SetValue(_api_manager->m_hmaterial, &(material), sizeof(_material)));

	application_throw_hr(_fx->Begin(NULL, 0));
	application_throw_hr(_fx->BeginPass(0));
	application_throw_hr(_api_manager->m_d3ddevice->DrawPrimitive(D3DPT_LINELIST, 0, cable_count ));
	application_throw_hr(_fx->EndPass());
	application_throw_hr(_fx->End()); 
	/***********************************************************************************************/

	/* |    |    |  */
	/* |    |    |  */
	/* |    |    |  */
	/* 0    0    0 support line buffer */
	application_throw_hr(!m_buffer->Lock(0, 0, (void**)&v, 0));
	for(uint32_t i=0,ii=0; ii<support_count;   ii++ ){
		v[i++] = m_supports[ii].m_particle->getposition();
		v[i++] = m_supports[ii].m_anchor;
	}
	application_throw_hr(!m_buffer->Unlock());

	/* draw supports *******************************************************************************/
	material.diffuse   = _vec4(0.7f, 0.7f, 0.7f,1.0f);
	application_throw_hr(_fx->SetValue(_api_manager->m_hmaterial, &(material), sizeof(_material)));

	application_throw_hr(_fx->Begin(NULL, 0));
	application_throw_hr(_fx->BeginPass(0));
	application_throw_hr(_api_manager->m_d3ddevice->DrawPrimitive(D3DPT_LINELIST, 0, support_count ));
	application_throw_hr(_fx->EndPass());
	application_throw_hr(_fx->End()); 
	/***********************************************************************************************/

	/* shader: object technique */
	application_error_hr(_fx->SetTechnique(_api_manager->m_htech) );

	particle_world::_particles &particles = m_world.m_particles;

	/* draw particles ******************************************************************************/
	for ( uint32_t i=0; i<m_world.m_particles.m_count; i++ ) {

		particle *_particle = m_world.m_particles[i];

		/* set particle size and position */
		mat =_scale( _vec3(0.2f,0.2f,0.2f) ) * _translate( _particle->getposition() );

		if(!_application->m_object_manager->drawsphere( mat, _vec4(0.0f,0.0f,0.0f,1.0f) ) ) { return false; }
	}
	/***********************************************************************************************/

	/* set  mass size and position */
	mat =_scale( _vec3(0.2f,0.2f,0.2f) ) * _translate( m_mass_display_pos );
	/* draw mass*/
	if(!_application->m_object_manager->drawsphere( mat, _vec4(1.0f,0.0f,0.0f,1.0f) ) ) { return false; }

	return true;
}

void bridge::updateadditionalmass() {

	/* set particle mass */
	for (uint32_t i = 0; i < 12; i++){
		m_particle_array[i].setmass(base_mass);
	}

	// find the coordinates of the mass as an index and proportion
	int x = int( m_mass_pos.x );
	float xp = fmod(m_mass_pos.x, float(1.0f));
	if (x < 0){
		x = 0;
		xp = 0;
	}
	if (x >= 5){
		x = 5;
		xp = 0;
	}

	int z = int( m_mass_pos.z );
	float zp = fmod( m_mass_pos.z, float(1.0f));
	if (z < 0){
		z = 0;
		zp = 0;
	}
	if (z >= 1){
		z = 1;
		zp = 0;
	}

	// calculate where to draw the mass
	m_mass_display_pos.clear();

	// add the proportion to the correct masses
	m_particle_array[x*2+z].setmass(base_mass + extra_mass*(1-xp)*(1-zp));
	m_mass_display_pos.addscaledvector(m_particle_array[x*2+z].getposition(), (1-xp)*(1-zp) );

	if (xp > 0) {
		m_particle_array[x*2+z+2].setmass(base_mass + extra_mass*xp*(1-zp));
		m_mass_display_pos.addscaledvector( m_particle_array[x*2+z+2].getposition(), xp*(1-zp) );

		if (zp > 0){
			m_particle_array[x*2+z+3].setmass(base_mass + extra_mass*xp*zp);
			m_mass_display_pos.addscaledvector(m_particle_array[x*2+z+3].getposition(), xp*zp );
		}
	}
	if (zp > 0){
		m_particle_array[x*2+z+1].setmass(base_mass + extra_mass*(1-xp)*zp);
		m_mass_display_pos.addscaledvector(m_particle_array[x*2+z+1].getposition(), (1-xp)*zp);
	}
}

void bridge::key( uint8_t key)
{
	switch(key)
	{
	case 's': case 'S':
		m_mass_pos.z += 0.1f;
		if ( m_mass_pos.z > 1.0f) m_mass_pos.z = 1.0f;
		break;
	case 'w': case 'W':
		m_mass_pos.z -= 0.1f;
		if (m_mass_pos.z < 0.0f) m_mass_pos.z = 0.0f;
		break;
	case 'a': case 'A':
		m_mass_pos.x -= 0.1f;
		if (m_mass_pos.x < 0.0f) m_mass_pos.x = 0.0f;
		break;
	case 'd': case 'D':
		m_mass_pos.x += 0.1f;
		if (m_mass_pos.x > 5.0f) m_mass_pos.x = 5.0f;
		break;
	}
}