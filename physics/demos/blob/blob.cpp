#include "blob.h"


#include "clock.h"
#include "application.h"
#include "d3d_manager.h"
#include "object_manager.h"


blob::blob() : m_xaxis(0), m_yaxis(0), m_world(platform_count+blob_count, platform_count) {
	m_blobs      = NULL;
	m_platforms  = NULL;
}

bool blob::init(){

	m_state_string = _string("A:left|D:right|W:up|S:down");

	m_buffer = NULL;

	/* platform buffer ************************************************/
	application_throw_hr(_api_manager->m_d3ddevice->CreateVertexBuffer(
		platform_count*2 * sizeof(_vec3),
		D3DUSAGE_WRITEONLY,0, D3DPOOL_MANAGED,	
		&m_buffer, 0));
	if(!m_buffer){ application_throw("line buffer"); }
	/******************************************************************/

	// create the blob storage
	m_blobs = new particle[blob_count];
	random r;

	// create the force generator
	m_blob_force_generator.m_particles = m_blobs;
	m_blob_force_generator.m_max_attraction = 20.0f;
	m_blob_force_generator.m_max_replusion = 10.0f;
	m_blob_force_generator.m_min_natural_distance = blob_radius*0.75f;
	m_blob_force_generator.m_max_natural_distance = blob_radius*1.5f;
	m_blob_force_generator.m_max_distance = blob_radius * 2.5f;
	m_blob_force_generator.m_max_float = 2;
	m_blob_force_generator.m_float_head = 8.0f;

	// create the platforms
	m_platforms = new platform[platform_count];
	for (uint32_t i = 0; i < platform_count; i++) {

		m_platforms[i].m_start = _vec3(
			float(i%2)*10.0f - 5.0f,
			float(i)*4.0f + ((i%2)?0.0f:2.0f),
			0);
		m_platforms[i].m_start.x += r.randombinomial(2.0f);
		m_platforms[i].m_start.y += r.randombinomial(2.0f);

		m_platforms[i].m_end = _vec3(
			float(i%2)*10.0f + 5.0f,
			float(i)*4.0f + ((i%2)?2.0f:0.0f),
			0);
		m_platforms[i].m_end.x += r.randombinomial(2.0f);
		m_platforms[i].m_end.y += r.randombinomial(2.0f);

		// make sure the platform knows which particles it
		// should collide with.
		m_platforms[i].m_particles = m_blobs;
		m_world.getcontactgenerators().pushback(m_platforms + i);
	}

	// create the blobs.
	platform *p = m_platforms + (platform_count-2);
	float fraction = (float)1.0 / blob_count;
	_vec3 delta = p->m_end - p->m_start;
	for (uint32_t i = 0; i < blob_count; i++)
	{
		uint32_t me = (i+blob_count/2) % blob_count;
		m_blobs[i].setposition(
			p->m_start + delta * (float(me)*0.8f*fraction+0.1f) +
			_vec3(0, 1.0f+r.randomreal(), 0));

		m_blobs[i].setvelocity(0,0,0);
		m_blobs[i].setdamping(0.2f);
		m_blobs[i].setacceleration( _utility::gravity * 0.4f);
		m_blobs[i].setmass(1.0f);
		m_blobs[i].clearaccumulator();

		m_world.m_particles.pushback(m_blobs + i);
		m_world.getforceregistry().add(m_blobs + i, &m_blob_force_generator);
	}

	return true;
}

void blob::clear(){

	if( m_platforms ) { delete [] m_platforms; }
	if( m_blobs )     { delete[] m_blobs; }
	application_releasecom(m_buffer);
}

bool blob::update(){

	// clear accumulators
	m_world.startframe();

	// find the duration of the last frame in seconds
	float duration = application_clock->m_last_frame_seconds;
	if (duration > 0.0f) { 

		// recenter the axes
		m_xaxis *= pow(0.1f, duration);
		m_yaxis *= pow(0.1f, duration);

		// move the controlled blob
		m_blobs[0].addforce(_vec3( m_xaxis, m_yaxis, 0)*10.0f);

		// run the simulation
		m_world.runphysics(duration);

		// bring all the particles back to 2d
		_vec3 position;
		for (uint32_t i = 0; i < blob_count; i++) {
			m_blobs[i].getposition(&position);
			position.z = 0.0f;
			m_blobs[i].setposition(position);
		}
	}

	return render();
}

bool blob::render(){

	_vec3 pos = m_blobs[0].getposition();
	//set camera view matrix
	_application->m_view = _lookatrh( _vec3(pos.x, pos.y, 6.0), _vec3(pos.x, pos.y, 0.0), _utility::up);

	_vec3 * v_ = 0;
	/* platform line buffer*/
	application_throw_hr( m_buffer->Lock(0, 0, (void**)&v_, 0));
	for(uint32_t i=0,ii=0; ii<platform_count;   ii++ ){
		v_[i++] = m_platforms[ii].m_start;
		v_[i++] = m_platforms[ii].m_end;
	}
	application_throw_hr( m_buffer->Unlock());

	/* draw platforms */

	// shader: flat (platforms) technique
	application_error_hr(_fx->SetTechnique(_api_manager->m_hflat_tech) );


	/* draw platforms ******************************************************************/
	_material material;
	material.diffuse = _vec4(0.0f,0.0f,1.0f,1.0f);

	_mat4 mat = _application->m_view*_application->m_projection;

	application_throw_hr(_fx->SetMatrix(_api_manager->m_hmvp,(D3DXMATRIX*)&mat ));

	application_throw_hr(_api_manager->m_d3ddevice->SetVertexDeclaration(_api_manager->m_flat_declaration));

	application_throw_hr(_api_manager->m_d3ddevice->SetStreamSource(0, m_buffer, 0, sizeof(_vec3)));

	application_throw_hr(_fx->SetValue(_api_manager->m_hmaterial, &(material), sizeof(_material)));

	application_throw_hr(_fx->Begin(NULL, 0));
	application_throw_hr(_fx->BeginPass(0));
	application_throw_hr(_api_manager->m_d3ddevice->DrawPrimitive(D3DPT_LINELIST, 0, platform_count ));
	application_throw_hr(_fx->EndPass());
	application_throw_hr(_fx->End()); 
	/***********************************************************************************/

	/* shader: object technique*/
	application_throw_hr(_fx->SetTechnique(_api_manager->m_htech) );

	/* draw blobs */
	for (uint32_t i = 0; i < blob_count; i++) {
		_vec3 &p = m_blobs[i].getposition();
		mat =_scale( _vec3((blob_radius*0.8f)*2,0.8f,0.8f) ) * _translate( m_world.m_particles[i]->getposition() );
		if( !_application->m_object_manager->drawsphere(mat, _vec4(1.0f,0.0f,0.0f,1.0f) ) ){ application_error("drawsphere"); }
	}

	_vec3 p = m_blobs[0].getposition();
	_vec3 v = m_blobs[0].getvelocity() * 0.05f;
	v.trim(blob_radius*0.5f);

	p = p + v;

	/* draw eyes */
	mat =_scale( _vec3( (blob_radius*0.2f)*2.0f,0.2f,0.2f) ) * _translate( _vec3(p.x-(blob_radius*0.25f), p.y, blob_radius) );
	if( !_application->m_object_manager->drawsphere(mat, _vec4(1.0f,1.0f,1.0f,1.0f) ) ){ application_throw("drawsphere"); }

	mat =_scale( _vec3( (blob_radius*0.1f)*2.0f,0.1f,0.1f) ) * _translate( _vec3(p.x-(blob_radius*0.25f), p.y, blob_radius*1.2f) );
	if( !_application->m_object_manager->drawsphere(mat, _vec4(0.0f,0.0f,0.0f,1.0f) ) ){ application_throw("drawsphere"); }

	mat =_scale( _vec3( (blob_radius*0.2f)*2.0f,0.2f,0.2f) ) * _translate( _vec3(p.x+(blob_radius*0.25f), p.y, blob_radius) );
	if( !_application->m_object_manager->drawsphere(mat, _vec4(1.0f,1.0f,1.0f,1.0f) ) ){ application_throw("drawsphere"); }

	mat =_scale( _vec3( (blob_radius*0.1f)*2.0f,0.1f,0.1f) ) * _translate( _vec3(p.x+(blob_radius*0.25f), p.y, blob_radius*1.2f) );
	if( !_application->m_object_manager->drawsphere(mat, _vec4(0.0f,0.0f,0.0f,1.0f) ) ){ application_throw("drawsphere"); }
	return true;
}

void blob::reset() {

	random r;
	platform *p = m_platforms + (platform_count-2);
	float fraction = (float)1.0 / blob_count;
	_vec3 delta = p->m_end - p->m_start;

	for (uint32_t i = 0; i < blob_count; i++) {
		uint32_t me = (i+blob_count/2) % blob_count;
		m_blobs[i].setposition( p->m_start+delta*(float(me)*0.8f*fraction+0.1f)+_vec3(0,1.0f+r.randomreal(),0));
		m_blobs[i].setvelocity(0,0,0);
		m_blobs[i].clearaccumulator();
	}

}

void blob::key( uint8_t key) {

	switch(key)
	{
	case 'w': case 'W':
		m_yaxis = 1.0;
		break;
	case 's': case 'S':
		m_yaxis = -1.0;
		break;
	case 'a': case 'A':
		m_xaxis = -1.0f;
		break;
	case 'd': case 'D':
		m_xaxis = 1.0f;
		break;
	case 'r': case 'R':
		reset();
		break;
	}
}
