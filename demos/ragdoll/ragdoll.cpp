#include "ragdoll.h"


#include "clock.h"
#include "application.h"
#include "d3d_manager.h"
#include "object_manager.h"



ragdoll::ragdoll(): 
m_resolver(maxcontacts*8),
	m_render_debug_info(false),
	m_pause_simulation(true),
	m_auto_pause_simulation(false)
{

}


bool ragdoll::init(){

	m_state_string = _string("reset: R run simulation: P render contacts: C advance frame: spcae");

	/* reference points buffer ************************************************************/
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
	/**************************************************************************************/


	/* reference line buffer *************************************************************/
	m_buffer = NULL;
	application_throw_hr(_api_manager->m_d3ddevice->CreateVertexBuffer(
		6 * sizeof(_vec3),
		D3DUSAGE_WRITEONLY,0, D3DPOOL_MANAGED,	
		&m_buffer, 0));
	if(!m_buffer){ application_throw("buffer"); }
	v = 0;
	application_throw_hr( m_buffer->Lock(0, 0, (void**)&v, 0));
	v[0]  = _vec3(-20,0,0);
	v[1]  = _vec3(20,0,0);
	v[2]  = _vec3(0,0,-20);
	v[3]  = _vec3(0,0,20);
	application_throw_hr( m_buffer->Unlock()); 
	/**************************************************************************************/

	m_cdata.m_contact_array = m_contacts;

	// Set up the bone hierarchy.

	// Right Knee
	m_joints[0].set(
		bones[0].m_body, _vec3(0, 1.07f, 0),
		bones[1].m_body, _vec3(0, -1.07f, 0),
		0.15f
		);

	// Left Knee
	m_joints[1].set(
		bones[2].m_body, _vec3(0, 1.07f, 0),
		bones[3].m_body, _vec3(0, -1.07f, 0),
		0.15f
		);

	// Right elbow
	m_joints[2].set(
		bones[9].m_body, _vec3(0, 0.96f, 0),
		bones[8].m_body, _vec3(0, -0.96f, 0),
		0.15f
		);

	// Left elbow
	m_joints[3].set(
		bones[11].m_body, _vec3(0, 0.96f, 0),
		bones[10].m_body, _vec3(0, -0.96f, 0),
		0.15f
		);

	// Stomach to Waist
	m_joints[4].set(
		bones[4].m_body, _vec3(0.054f, 0.50f, 0),
		bones[5].m_body, _vec3(-0.043f, -0.45f, 0),
		0.15f
		);

	m_joints[5].set(
		bones[5].m_body, _vec3(-0.043f, 0.411f, 0),
		bones[6].m_body, _vec3(0, -0.411f, 0),
		0.15f
		);

	m_joints[6].set(
		bones[6].m_body, _vec3(0, 0.521f, 0),
		bones[7].m_body, _vec3(0, -0.752f, 0),
		0.15f
		);

	// Right hip
	m_joints[7].set(
		bones[1].m_body, _vec3(0, 1.066f, 0),
		bones[4].m_body, _vec3(0, -0.458f, -0.5f),
		0.15f
		);

	// Left Hip
	m_joints[8].set(
		bones[3].m_body, _vec3(0, 1.066f, 0),
		bones[4].m_body, _vec3(0, -0.458f, 0.5f),
		0.105f
		);

	// Right shoulder
	m_joints[9].set(
		bones[6].m_body, _vec3(0, 0.367f, -0.8f),
		bones[8].m_body, _vec3(0, 0.888f, 0.32f),
		0.15f
		);

	// Left shoulder
	m_joints[10].set(
		bones[6].m_body, _vec3(0, 0.367f, 0.8f),
		bones[10].m_body, _vec3(0, 0.888f, -0.32f),
		0.15f
		);

	_application->addflags(application_spherical_cam);
	_application->m_distance = 30.0f;
	_application->m_pitch        = -35.0f;
	_application->m_pitch_pos    = -35.0f;
	_application->m_yaw          =  90.0f;
	_application->m_yaw_pos      =  90.0f;

	// set up the initial positions
	reset();

	return true;
}

void ragdoll::generatecontacts() {

	// create the ground plane data
	collision_plane plane;
	plane.m_direction = _vec3(0,1,0);
	plane.m_offset = 0;

	// set up the collision data structure
	m_cdata.reset(maxcontacts);
	m_cdata.m_friction = 0.9f;
	m_cdata.m_restitution = 0.6f;
	m_cdata.m_tolerance = 0.1f;

	// perform exhaustive collision detection on the ground plane
	_mat4 transform, othertransform;
	_vec3 position, otherposition;
	for (bone *bone_ = bones; bone_ < bones+NUM_BONES; bone_++) {
		// check for collisions with the ground plane
		if (!m_cdata.hasmorecontacts()) { return; }
		collision_detector::boxandhalfspace(*bone_, plane, &m_cdata);

		collision_sphere bonesphere = bone_->getcollisionsphere();

		// check for collisions with each other box
		for (bone * other = bone_+1; other < bones+NUM_BONES; other++)
		{
			if (!m_cdata.hasmorecontacts()) { return; }

			collision_sphere othersphere = other->getcollisionsphere();

			collision_detector::sphereandsphere(
				bonesphere,
				othersphere,
				&m_cdata
				);
		}
	}

	// check for joint violation
	for (joint *joint = m_joints; joint < m_joints+NUM_JOINTS; joint++) {
		if (!m_cdata.hasmorecontacts()) { return; }
		uint32_t added = joint->addcontact( m_cdata.m_contacts, m_cdata.m_contacts_left );
		m_cdata.addcontacts(added);
	}
}

void ragdoll::clear(){
	application_releasecom(m_buffer);
	application_releasecom(m_point_buffer);
}

void ragdoll::reset() {

	// reset the bone hierarchy.
	bones[0].setstate(
		_vec3(0.0f, 0.993f, -0.5f),
		_vec3(0.301f, 1.0f, 0.234f));
	bones[1].setstate(
		_vec3(0.0f, 3.159f, -0.56f),
		_vec3(0.301f, 1.0f, 0.234f));
	bones[2].setstate(
		_vec3(0.0f, 0.993f, 0.5f),
		_vec3(0.301f, 1.0f, 0.234f));
	bones[3].setstate(
		_vec3(0.0f, 3.15f, 0.56f),
		_vec3(0.301f, 1.0f, 0.234f));
	bones[4].setstate(
		_vec3(-0.054f, 4.683f, 0.013f),
		_vec3(0.415f, 0.392f, 0.690f));
	bones[5].setstate(
		_vec3(0.043f, 5.603f, 0.013f),
		_vec3(0.301f, 0.367f, 0.693f));
	bones[6].setstate(
		_vec3(0.0f, 6.485f, 0.013f),
		_vec3(0.435f, 0.367f, 0.786f));
	bones[7].setstate(
		_vec3(0.0f, 7.759f, 0.013f),
		_vec3(0.45f, 0.598f, 0.421f));
	bones[8].setstate(
		_vec3(0.0f, 5.946f, -1.066f),
		_vec3(0.267f, 0.888f, 0.207f));
	bones[9].setstate(
		_vec3(0.0f, 4.024f, -1.066f),
		_vec3(0.267f, 0.888f, 0.207f));
	bones[10].setstate(
		_vec3(0.0f, 5.946f, 1.066f),
		_vec3(0.267f, 0.888f, 0.207f));
	bones[11].setstate(
		_vec3(0.0f, 4.024f, 1.066f),
		_vec3(0.267f, 0.888f, 0.207f));

	float strength = -m_random.randomreal(500.0f, 1000.0f);
	
	for ( uint32_t i = 0; i < NUM_BONES; i++){
		bones[i].m_body->addforceatbodypoint( _vec3(strength, 0, 0), _vec3() );
	}
	bones[6].m_body->addforceatbodypoint(
		_vec3(strength, 0, m_random.randombinomial(1000.0f)),
		_vec3(m_random.randombinomial(4.0f), m_random.randombinomial(3.0f), 0)
		);
	// reset the contacts
	m_cdata.m_contact_count = 0;
}

void ragdoll::updateobjects( float duration)
{
	for (bone *bone = bones; bone < bones+NUM_BONES; bone++) {
		bone->m_body->integrate(duration);
		bone->calculateinternals();
	}
}

bool ragdoll::update(){

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

bool ragdoll::render(){

	// shader: flat (lines) technique
	application_error_hr(_fx->SetTechnique(_api_manager->m_hflat_tech) );

	_mat4 mat = _application->m_view*_application->m_projection;

	application_throw_hr(_fx->SetMatrix(_api_manager->m_hmvp,(D3DXMATRIX*)&mat ));

	application_throw_hr(_api_manager->m_d3ddevice->SetVertexDeclaration(_api_manager->m_flat_declaration));

	application_throw_hr(_api_manager->m_d3ddevice->SetStreamSource(0,m_buffer, 0, sizeof(_vec3)));


	/* draw joints (lines) ************************************************************/
	application_throw_hr(_fx->Begin(NULL, 0));

	_material material;

	for (uint32_t pos = 0,i = 0; i < NUM_JOINTS; i++) {
		joint *joint_ = m_joints + i;
		_vec3 a_pos = joint_->m_body[0]->getpointinworldspace(joint_->m_position[0]);
		_vec3 b_pos = joint_->m_body[1]->getpointinworldspace(joint_->m_position[1]);
		float length = (b_pos - a_pos).magnitude();

		if (length > joint_->m_error) { material.diffuse = _vec4(1,0,0,1); }
		else {material.diffuse = _vec4(0,1,0,1); }


		application_throw_hr(_fx->SetValue(_api_manager->m_hmaterial, &(material), sizeof(_material)));


		_vec3 * v = 0;
		application_throw_hr(m_buffer->Lock(0, 0, (void**)&v, 0));
		v[4]  = _vec3(a_pos.x, a_pos.y, a_pos.z);
		v[5]  = _vec3(b_pos.x, b_pos.y, b_pos.z);

		application_throw_hr(m_buffer->Unlock()); 

		application_throw_hr(_fx->BeginPass(0));
		application_throw_hr(_api_manager->m_d3ddevice->DrawPrimitive(D3DPT_LINELIST, 4, 1));
		application_throw_hr(_fx->EndPass());

	}
	application_throw_hr(_fx->End()); 
	/**********************************************************************************/

	/* draw ground lines **************************************************************/
	material.diffuse = _vec4(0.02f,0.02f,0.02f,1);
	application_throw_hr(_fx->SetValue(_api_manager->m_hmaterial, &(material), sizeof(_material)));
	application_throw_hr(_fx->Begin(NULL, 0));
	application_throw_hr(_fx->BeginPass(0));
	application_throw_hr(_api_manager->m_d3ddevice->DrawPrimitive(D3DPT_LINELIST, 0, 2));
	application_throw_hr(_fx->EndPass());
	application_throw_hr(_fx->End()); 
	/**********************************************************************************/


	/* draw circles *******************************************************************/
	application_throw_hr(_api_manager->m_d3ddevice->SetStreamSource(0,m_point_buffer, 0, sizeof(_vec3)));
	application_throw_hr(_fx->Begin(NULL, 0));
	application_throw_hr(_fx->BeginPass(0));
	application_throw_hr(_api_manager->m_d3ddevice->DrawPrimitive(D3DPT_POINTLIST, 0, 3600));
	application_throw_hr(_fx->EndPass());
	application_throw_hr(_fx->End()); 
	/**********************************************************************************/

	/* shader: object technique */
	application_error_hr(_fx->SetTechnique(_api_manager->m_htech) );

	/* draw bones *********************************************************************/
	_mat4 world;
	for (uint32_t i = 0; i < NUM_BONES; i++)  {
		_vec3 scale = _vec3(bones[i].m_half_size.x*2, bones[i].m_half_size.y*2, bones[i].m_half_size.z*2);
		_mat4 world =  _scale(scale) * bones[i].m_body->gettransform();

		if (bones[i].m_body->getawake()) { material.diffuse = _vec4(0.5f, 0.3f, 0.3f,1.0f ); }
		else { material.diffuse = _vec4(0.3f, 0.3f, 0.5f,1.0f); }
		_application->m_object_manager->drawcube( world , material.diffuse );

	}
	/**********************************************************************************/

	/* draw contacts if debug is toggled*/
	if(!drawdebug()){ return false;}
	return drawdebug(); 
}

bool ragdoll::drawdebug() {

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
void ragdoll::key(uint8_t key) {

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