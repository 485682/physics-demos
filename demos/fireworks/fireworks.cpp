#include "fireworks.h"

#include "clock.h"
#include "application.h"
#include "d3d_manager.h"
#include "object_manager.h"


void fireworks::create(uint32_t type, const firework *parent){
    // get the rule needed to create this firework
    firework_rule *rule = rules + (type - 1);

    // create the firework
    rule->create( m_firework_buffer+m_next_firework, parent);

    // increment the index for the next firework
    m_next_firework = (m_next_firework + 1) % max_fireworks;
}

void fireworks::create(uint32_t type, uint32_t number, const firework *parent){
    for (uint32_t i = 0; i < number; i++) { create(type, parent); }
}

void fireworks::initfireworkrules() {

	// go through the firework types and create their rules.
	rules[0].init(2);
	rules[0].setParameters(
		1, // type
		0.5f, 1.4f, // age range
		_vec3(-5.0f, 25.0f, -5.0f), // min velocity
		_vec3(5.0f, 28.0f, 5.0f), // max velocity
		0.1f // damping
		);
	rules[0].m_payloads[0].set(3, 5);
	rules[0].m_payloads[1].set(5, 5);

	rules[1].init(1);
	rules[1].setParameters(
		2, // type
		0.5f, 1.0f, // age range
		_vec3(-5.0f, 10.0f, -5.0f), // min velocity
		_vec3(5.0f, 20.0f, 5.0f), // max velocity
		0.8f // damping
		);
	rules[1].m_payloads[0].set(4, 2);

	rules[2].init(0);
	rules[2].setParameters(
		3, // type
		0.5f, 1.5f, // age range
		_vec3(-5.0f, -5.0f, -5.0f), // min velocity
		_vec3(5.0f, 5.0f, 5.0f), // max velocity
		0.1f // damping
		);

	rules[3].init(0);
	rules[3].setParameters(
		4, // type
		0.25f, 0.5f, // age range
		_vec3(-20.0f, 5.0f, -5.0f), // min velocity
		_vec3(20.0f, 5.0f, 5.0f), // max velocity
		0.2f // damping
		);

	rules[4].init(1);
	rules[4].setParameters(
		5, // type
		0.5f, 1.0f, // age range
		_vec3(-20.0f, 2.0f, -5.0f), // min velocity
		_vec3(20.0f, 18.0f, 5.0f), // max velocity
		0.01f // damping
		);
	rules[4].m_payloads[0].set(3, 5);

	rules[5].init(0);
	rules[5].setParameters(
		6, // type
		3.0f, 5.0f, // age range
		_vec3(-5.0f, 5.0f, -5.0f), // min velocity
		_vec3(5.0f, 10.0f, 5.0f), // max velocity
		0.95f // damping
		);

	rules[6].init(1);
	rules[6].setParameters(
		7, // type
		4.0f, 5.0f, // age range
		_vec3(-5.0f, 50.0f, -5.0f), // min velocity
		_vec3(5.0f, 60.0f, 5.0f), // max velocity
		0.01f // damping
		);
	rules[6].m_payloads[0].set(8, 10);

	rules[7].init(0);
	rules[7].setParameters(
		8, // type
		0.25f, 0.5f, // age range
		_vec3(-1.0f, -1.0f, -1.0f), // min velocity
		_vec3(1.0f, 1.0f, 1.0f), // max velocity
		0.01f // damping
		);

	rules[8].init(0);
	rules[8].setParameters(
		9, // type
		3.0f, 5.0f, // age range
		_vec3(-15.0f, 10.0f, -5.0f), // min velocity
		_vec3(15.0f, 15.0f, 5.0f), // max velocity
		0.95f // damping
		);
	// ... and so on for other firework types ...
}

bool fireworks::init(){

	m_state_string = _string("fire firewoks: key 1 - 9");

	m_buffer = NULL;

	/* firework buffer *************************************************/
	application_throw_hr(_api_manager->m_d3ddevice->CreateVertexBuffer(
		12 * sizeof(_vec3),
		D3DUSAGE_WRITEONLY,0, D3DPOOL_MANAGED,	
		&m_buffer, 0));
	if(!m_buffer){ application_throw("buffer"); }
	/*******************************************************************/


	m_next_firework = 0;
	
	// Make all shots unused
	for (
		firework *_firework = m_firework_buffer; 
		_firework < m_firework_buffer+max_fireworks;
	_firework++)
	{
		_firework->m_type = 0;
	}

	// Create the firework types
	initfireworkrules();

	return true;
}

bool fireworks::render(){

	//set camera view matrix
	_application->m_view =_lookatrh(_vec3(0.0, 4.0, -20.0),_vec3(-0.0, 4.0, 10.0), _utility::up);

	/* shader: (flat) firework technique */
	application_error_hr(_fx->SetTechnique(_api_manager->m_hflat_tech) );

	_mat4 mat = _application->m_view*_application->m_projection;

	application_throw_hr(_fx->SetMatrix(_api_manager->m_hmvp,(D3DXMATRIX*)&mat ));

	application_throw_hr(_api_manager->m_d3ddevice->SetVertexDeclaration(_api_manager->m_flat_declaration));

	application_throw_hr(_api_manager->m_d3ddevice->SetStreamSource(0,m_buffer, 0, sizeof(_vec3)));

	_material material;
	const static float size = 0.1f;
	application_throw_hr(_fx->Begin(NULL, 0));

	// Render each firework in turn
	for (firework *_firework = m_firework_buffer;
		_firework < m_firework_buffer+max_fireworks;
		_firework++)
	{
		// check if we need to process this firework.
		if (_firework->m_type > 0)
		{
			//fire work color
			switch (_firework->m_type)
			{
			case 1: material.diffuse = _vec4(1.0f,0.0f,0.0f,1.0f); break;
			case 2: material.diffuse = _vec4(1.0f,0.5f,0.0f,1.0f); break;
			case 3: material.diffuse = _vec4(1.0f,1.0f,0.0f,1.0f); break;
			case 4: material.diffuse = _vec4(0.0f,1.0f,0.0f,1.0f); break;
			case 5: material.diffuse = _vec4(0.0f,1.0f,1.0f,1.0f); break;
			case 6: material.diffuse = _vec4(0.4f,0.4f,1.0f,1.0f); break;
			case 7: material.diffuse = _vec4(1.0f,0.0f,1.0f,1.0f); break;
			case 8: material.diffuse = _vec4(1.0f,1.0f,1.0f,1.0f); break;
			case 9: material.diffuse = _vec4(1.0f,0.5f,0.5f,1.0f); break;
			};
		
			application_throw_hr(_fx->SetValue(_api_manager->m_hmaterial, &(material), sizeof(_material)));

			const _vec3 &pos = _firework->getposition();


			_vec3 * v = 0;
			/* firework buffer (quads) ********************/
			application_throw_hr(m_buffer->Lock(0, 0, (void**)&v, 0));
			/* firework quad*/
			v[0]  = _vec3(pos.x-size, pos.y-size, pos.z);
			v[1]  = _vec3(pos.x+size, pos.y-size, pos.z);
			v[2]  = _vec3(pos.x-size, pos.y+size, pos.z);
			v[3]  = _vec3(pos.x-size, pos.y+size, pos.z);
			v[4]  = _vec3(pos.x+size, pos.y-size, pos.z);
			v[5]  = _vec3(pos.x+size, pos.y+size, pos.z);


			/* mirror quad*/
			v[6]  = _vec3(pos.x-size, -pos.y-size, pos.z);
			v[7]  = _vec3(pos.x+size, -pos.y-size, pos.z);
			v[8]  = _vec3(pos.x-size, -pos.y+size, pos.z);
			v[9]  = _vec3(pos.x-size, -pos.y+size, pos.z);
			v[10] = _vec3(pos.x+size, -pos.y-size, pos.z);
			v[11] = _vec3(pos.x+size, -pos.y+size, pos.z);
			application_throw_hr(m_buffer->Unlock()); 
			/**********************************************/

			/** draw quads*/
			application_throw_hr(_fx->BeginPass(0));
			application_throw_hr(_api_manager->m_d3ddevice->DrawPrimitive(D3DPT_TRIANGLELIST, 0, 4));
			application_throw_hr(_fx->EndPass());
		}
	}
	application_throw_hr(_fx->End()); 
	return true;
}
void fireworks::clear(){
	application_releasecom(m_buffer);
}

bool fireworks::update(){

	// find the duration of the last frame in seconds
    float duration = application_clock->m_last_frame_seconds;
	if (duration <= 0.0f) { return render(); };

    for (firework *_firework = m_firework_buffer;
         _firework < m_firework_buffer+max_fireworks;
         _firework++)
    {
        // check if we need to process this firework.
        if (_firework->m_type > 0)
        {
            // does it need removing?
            if (_firework->update(duration)) {
                // find the appropriate rule
                firework_rule *rule = rules + (_firework->m_type-1);

                // delete the current firework (this doesn't affect its
                // position and velocity for passing to the create function,
                // just whether or not it is processed for rendering or
                // physics.
                _firework->m_type = 0;

                // add the payload
                for (uint32_t i = 0; i < rule->m_payload_count; i++) {
                    firework_rule::payload * payload = rule->m_payloads + i;
                    create(payload->m_type, payload->m_count, _firework);
                }
            }
        }
    }

	return render();
}

void fireworks::key(uint8_t key){
	
    switch (key)
    {

		
    case '1': create(1, 1, NULL); break;
    case '2': create(2, 1, NULL); break;
    case '3': create(3, 1, NULL); break;
    case '4': create(4, 1, NULL); break;
    case '5': create(5, 1, NULL); break;
    case '6': create(6, 1, NULL); break;
    case '7': create(7, 1, NULL); break;
    case '8': create(8, 1, NULL); break;
    case '9': create(9, 1, NULL); break;
    }
}