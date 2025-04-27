#include "platform.h"


#include "clock.h"
#include "d3d_manager.h"

#include "object_manager.h"

#include "application.h"

platform::platform() : m_world(60) , m_rods(0),m_mass_position(0,0,0.5f) {

	m_rods = NULL;
	m_particle_array = NULL;
}

void platform::updateadditionalmass() {

	/* set particle masses */
	for (uint32_t i = 2; i < 6; i++) { m_particle_array[i].setmass(base_mass); }

	// Find the coordinates of the mass as an index and proportion
	float xp = m_mass_position.x;
	if (xp < 0) xp = 0;
	if (xp > 1) xp = 1;

	float zp = m_mass_position.z;
	if (zp < 0) zp = 0;
	if (zp > 1) zp = 1;

	// Calculate where to draw the mass

	m_mass_display_position.clear();

	// Add the proportion to the correct masses
	m_particle_array[2].setmass(base_mass + extra_mass*(1-xp)*(1-zp));
	m_mass_display_position.addscaledvector(
		m_particle_array[2].getposition(), (1-xp)*(1-zp)
		);

	if (xp > 0)
	{
		m_particle_array[4].setmass(base_mass + extra_mass*xp*(1-zp));
		m_mass_display_position.addscaledvector(
			m_particle_array[4].getposition(), xp*(1-zp)
			);

		if (zp > 0)
		{
			m_particle_array[5].setmass(base_mass + extra_mass*xp*zp);
			m_mass_display_position.addscaledvector(
				m_particle_array[5].getposition(), xp*zp
				);
		}
	}
	if (zp > 0)
	{
		m_particle_array[3].setmass(base_mass + extra_mass*(1-xp)*zp);
		m_mass_display_position.addscaledvector(
			m_particle_array[3].getposition(), (1-xp)*zp
			);
	}
}


bool platform::init(){

	m_state_string = _string("left: A key, right: D key up: W key down: S Key");

	/* line buffer (rods)**************************************************************/
	m_buffer = NULL;
	application_throw_hr(_api_manager->m_d3ddevice->CreateVertexBuffer(
		(rod_count*2) * sizeof(_vec3),
		D3DUSAGE_WRITEONLY,0, D3DPOOL_MANAGED,	
		&m_buffer, 0));
	if(!m_buffer){ application_throw("line buffer"); }
	/**********************************************************************************/


	/* allocate particles*/
	m_particle_array = new particle[6];
	for (uint32_t i = 0; i < 6; i++) {
		m_world.m_particles.pushback( m_particle_array + i);
	}

	/* init ground contact generator */
	m_ground_contact_generator.init( &(m_world.m_particles) );
	m_world.getcontactgenerators().pushback(&m_ground_contact_generator);

	// Create the masses and connections.
	m_particle_array[0].setposition(0,0,1);
	m_particle_array[1].setposition(0,0,-1);
	m_particle_array[2].setposition(-3,2,1);
	m_particle_array[3].setposition(-3,2,-1);
	m_particle_array[4].setposition(4,2,1);
	m_particle_array[5].setposition(4,2,-1);
	for (uint32_t i = 0; i < 6; i++) {
		m_particle_array[i].setmass(base_mass);
		m_particle_array[i].setvelocity(0,0,0);
		m_particle_array[i].setdamping(0.9f);
		m_particle_array[i].setacceleration(_utility::gravity);
		m_particle_array[i].clearaccumulator();
	}

	m_rods = new particle_rod[rod_count];

	m_rods[0].m_contact_particles[0] = &m_particle_array[0];
	m_rods[0].m_contact_particles[1] = &m_particle_array[1];
	m_rods[0].m_length = 2.0f;
	m_rods[1].m_contact_particles[0] = &m_particle_array[2];
	m_rods[1].m_contact_particles[1] = &m_particle_array[3];
	m_rods[1].m_length = 2.0f;
	m_rods[2].m_contact_particles[0] = &m_particle_array[4];
	m_rods[2].m_contact_particles[1] = &m_particle_array[5];
	m_rods[2].m_length = 2.0f;

	m_rods[3].m_contact_particles[0] = &m_particle_array[2];
	m_rods[3].m_contact_particles[1] = &m_particle_array[4];
	m_rods[3].m_length = 7.0f;
	m_rods[4].m_contact_particles[0] = &m_particle_array[3];
	m_rods[4].m_contact_particles[1] = &m_particle_array[5];
	m_rods[4].m_length = 7.0f;

	m_rods[5].m_contact_particles[0] = &m_particle_array[0];
	m_rods[5].m_contact_particles[1] = &m_particle_array[2];
	m_rods[5].m_length = 3.606f;
	m_rods[6].m_contact_particles[0] = &m_particle_array[1];
	m_rods[6].m_contact_particles[1] = &m_particle_array[3];
	m_rods[6].m_length = 3.606f;

	m_rods[7].m_contact_particles[0] = &m_particle_array[0];
	m_rods[7].m_contact_particles[1] = &m_particle_array[4];
	m_rods[7].m_length = 4.472f;
	m_rods[8].m_contact_particles[0] = &m_particle_array[1];
	m_rods[8].m_contact_particles[1] = &m_particle_array[5];
	m_rods[8].m_length = 4.472f;

	m_rods[9].m_contact_particles[0] = &m_particle_array[0];
	m_rods[9].m_contact_particles[1] = &m_particle_array[3];
	m_rods[9].m_length = 4.123f;
	m_rods[10].m_contact_particles[0] = &m_particle_array[2];
	m_rods[10].m_contact_particles[1] = &m_particle_array[5];
	m_rods[10].m_length = 7.28f;
	m_rods[11].m_contact_particles[0] = &m_particle_array[4];
	m_rods[11].m_contact_particles[1] = &m_particle_array[1];
	m_rods[11].m_length = 4.899f;
	m_rods[12].m_contact_particles[0] = &m_particle_array[1];
	m_rods[12].m_contact_particles[1] = &m_particle_array[2];
	m_rods[12].m_length = 4.123f;
	m_rods[13].m_contact_particles[0] = &m_particle_array[3];
	m_rods[13].m_contact_particles[1] = &m_particle_array[4];
	m_rods[13].m_length = 7.28f;
	m_rods[14].m_contact_particles[0] = &m_particle_array[5];
	m_rods[14].m_contact_particles[1] = &m_particle_array[0];
	m_rods[14].m_length = 4.899f;

	for (uint32_t i = 0; i < rod_count; i++){
		m_world.getcontactgenerators().pushback( &m_rods[i] );
	}

	updateadditionalmass();

	return true;
}

void platform::clear(){

	if( m_particle_array ) { delete [] m_particle_array; }
	if( m_rods )           { delete [] m_rods; }
}

bool platform::update(){

	// Clear accumulators
	m_world.startframe();

	// Find the duration of the last frame in seconds
	float duration = application_clock->m_last_frame_seconds;
	if (duration > 0.0f) { 

		// Run the simulation
		m_world.runphysics(duration);

		updateadditionalmass();
	}

	return render();
}

bool platform::render(){

	/* camera view matrix*/
	_application->m_view = _lookatrh( _vec3(0.0f, 10.0f, 20.0f), _vec3(0.0f, 0.0f, 0.0f), _utility::up);

	/* shader: line (rods) technique */
	application_error_hr(_fx->SetTechnique(_api_manager->m_hflat_tech) );

	_vec3 * v = 0;
	/* rod buffer */
	application_throw_hr( m_buffer->Lock(0, 0, (void**)&v, 0));
	for(uint32_t i=0,ii=0; ii<rod_count;   ii++ ){
		v[i++] = m_rods[ii].m_contact_particles[0]->getposition();
		v[i++] = m_rods[ii].m_contact_particles[1]->getposition();
	}
	application_throw_hr( m_buffer->Unlock());

	/* draw rods **********************************************************************/
	_material material;
	material.diffuse   = _vec4(0.0f,0.0f,1.0f,1.0f);

	_mat4 mat = _application->m_view*_application->m_projection;

	application_throw_hr(_fx->SetMatrix(_api_manager->m_hmvp,(D3DXMATRIX*)&mat ));

	application_throw_hr(_api_manager->m_d3ddevice->SetVertexDeclaration(_api_manager->m_flat_declaration));

	application_throw_hr(_api_manager->m_d3ddevice->SetStreamSource(0,m_buffer, 0, sizeof(_vec3)));

	application_throw_hr(_fx->SetValue(_api_manager->m_hmaterial, &(material), sizeof(_material)));

	application_throw_hr(_fx->Begin(NULL, 0));
	application_throw_hr(_fx->BeginPass(0));
	application_throw_hr(_api_manager->m_d3ddevice->DrawPrimitive(D3DPT_LINELIST, 0, rod_count ));
	application_throw_hr(_fx->EndPass());
	application_throw_hr(_fx->End()); 
	/**********************************************************************************/

	/* shader: object technique */
	application_error_hr(_fx->SetTechnique(_api_manager->m_htech) );

	for ( uint32_t i=0;i<m_world.m_particles.m_count;i++ )  {

		/* set particle size and position */
		mat =_scale( _vec3(0.2f,0.2f,0.2f) ) * _translate(m_world.m_particles[i]->getposition() );

		/* draw particles */
		if( !_application->m_object_manager->drawsphere(mat, _vec4(0.0f,0.0f,0.0f,1.0f) ) ){ application_throw("drawsphere"); }
	}

	/* set mass size and position */
	mat =_scale( _vec3(0.5f,0.5f,0.5f) ) * _translate( m_mass_display_position );

	/* draw mass */
	if( !_application->m_object_manager->drawsphere(mat, _vec4(1.0f,0.0f,0.0f,1.0f) ) ){ application_throw("drawsphere"); }

	return true;
}

void platform::key( uint8_t key) {
	switch(key)
	{
	case 'w': case 'W':
		m_mass_position.z += 0.1f;
		if (m_mass_position.z > 1.0f) { m_mass_position.z = 1.0f; }
		break;
	case 's': case 'S':
		m_mass_position.z -= 0.1f;
		if (m_mass_position.z < 0.0f) { m_mass_position.z = 0.0f; }
		break;
	case 'a': case 'A':
		m_mass_position.x -= 0.1f;
		if (m_mass_position.x < 0.0f) { m_mass_position.x = 0.0f; }
		break;
	case 'd': case 'D':
		m_mass_position.x += 0.1f;
		if (m_mass_position.x > 1.0f) { m_mass_position.x = 1.0f; }
		break;
	}
}