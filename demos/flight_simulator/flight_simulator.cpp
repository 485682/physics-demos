#include "flight_simulator.h"


#include "clock.h"
#include "application.h"
#include "d3d_manager.h"
#include "object_manager.h"



flight_simulator::flight_simulator():m_right_wing(_mat3(0,0,0, -1,-0.5f,0, 0,0,0),
	_mat3(0,0,0, -0.995f,-0.5f,0, 0,0,0),
	_mat3(0,0,0, -1.005f,-0.5f,0, 0,0,0),
	_vec3(-1.0f, 0.0f, 2.0f), &m_wind_speed),
	m_left_wing(
	_mat3(0,0,0, -1,-0.5f,0, 0,0,0),
	_mat3(0,0,0, -0.995f,-0.5f,0, 0,0,0),
	_mat3(0,0,0, -1.005f,-0.5f,0, 0,0,0),
	_vec3(-1.0f, 0.0f, -2.0f), &m_wind_speed),
	m_rudder(_mat3(0,0,0, 0,0,0, 0,0,0),
	_mat3(0,0,0, 0,0,0, 0.01f,0,0),
	_mat3(0,0,0, 0,0,0, -0.01f,0,0),
	_vec3(2.0f, 0.5f, 0), &m_wind_speed),
	m_tail(_mat3(0,0,0, -1,-0.5f,0, 0,0,-0.1f),
	_vec3(2.0f, 0, 0), &m_wind_speed),
	m_left_wing_control(0), m_right_wing_control(0), m_rudder_control(0),
	m_wind_speed(0,0,0) {  }

void flight_simulator::resetplane() {
	m_aircraft.setposition(0, 0, 0);
	m_aircraft.setorientation(1,0,0,0);

	m_aircraft.setvelocity(0,0,0);
	m_aircraft.setrotation(0,0,0);

	m_left_wing_control   = 0.0f;
	m_right_wing_control  = 0.0f;;
	m_rudder_control      = 0.0f;;

}

bool flight_simulator::init(){

	m_buffer = NULL;

	/*ground plane quads ************************************************/
	application_throw_hr(_api_manager->m_d3ddevice->CreateVertexBuffer(
		11000 * sizeof(_vec3),
		D3DUSAGE_WRITEONLY,0, D3DPOOL_MANAGED,	
		&m_buffer, 0));
	if(!m_buffer){ application_throw("line buffer"); }
	/********************************************************************/


	// Set up the aircraft rigid body.
	resetplane();

	/* set up the aircraft's rigid body ********/
	m_aircraft.setmass(2.5f);
	_mat3 it;
	it.setblockinertiatensor(_vec3(2,1,1), 1);
	m_aircraft.setinertiatensor(it);

	m_aircraft.setdamping(0.8f, 0.8f);

	m_aircraft.setacceleration(_utility::gravity);
	m_aircraft.calculatederiveddata();

	m_aircraft.setawake();
	m_aircraft.setcansleep(false);
	/****************************************/

    /* add force generators to registry *****/
	m_registry.add(&m_aircraft, &m_left_wing);
	m_registry.add(&m_aircraft, &m_right_wing);
	m_registry.add(&m_aircraft, &m_rudder);
	m_registry.add(&m_aircraft, &m_tail);
	/****************************************/

	return true;
}

void flight_simulator::clear(){
	application_releasecom(m_buffer);
}

bool flight_simulator::update(){

	// find the duration of the last frame in seconds
	float duration = application_clock->m_last_frame_seconds;
	if (duration > 0.0f){

		// start with no forces or acceleration.
		m_aircraft.clearaccumulators();

		// add the propeller force
		_vec3 propulsion(-10.0f, 0, 0);
		propulsion = m_aircraft.gettransform().transformdirection(propulsion);
		m_aircraft.addforce(propulsion);

		// add the forces acting on the aircraft.
		m_registry.updateforces(duration);

		// update the aircraft's physics.
		m_aircraft.integrate(duration);

		// do a very basic collision detection and response with the ground.
		_vec3 pos = m_aircraft.getposition();
		if (pos.y < 0.0f) {
			pos.y = 0.0f;
			m_aircraft.setposition(pos);

			if (m_aircraft.getvelocity().y < -10.0f) {
				resetplane();
			}
		}
	}

	m_state_string = _string("wings:w,a,s,d rudder:q,e clear:x reset:r alt:");
	m_state_string = m_state_string+_utility::floattostring(m_aircraft.getposition().y,true);

	m_state_string = m_state_string+_string(" spd: ");
	m_state_string = m_state_string+_utility::floattostring(m_aircraft.getvelocity().magnitude(),true);

	m_state_string = m_state_string+_string(" left: ");
	m_state_string = m_state_string+_utility::floattostring(m_left_wing_control,true);

	m_state_string = m_state_string+_string(" right: ");
	m_state_string = m_state_string+_utility::floattostring(m_right_wing_control,true);

	m_state_string = m_state_string+_string(" rudder: ");
	m_state_string = m_state_string+_utility::floattostring(m_rudder_control,true);


	return render();
}

bool flight_simulator::render(){


	_vec3 pos = m_aircraft.getposition();

	_vec3 offset(5.0f+m_aircraft.getvelocity().magnitude(), 0, 0);

	offset = m_aircraft.gettransform().transformdirection(offset);

	/* camera view matrix */
	_application->m_view =_lookatrh( 
		_vec3(pos.x+offset.x, pos.y+10.0f, pos.z+offset.z),
		_vec3(pos.x, pos.y, pos.z), 
		_utility::up);

	_material material;
	material.diffuse = _vec4(0.6f,0.6f,0.6f,1.0f);

	// shader: flat (quads) technique
	application_error_hr(_fx->SetTechnique(_api_manager->m_hflat_tech) );

	_mat4 mat = _application->m_view*_application->m_projection;

	application_throw_hr(_fx->SetMatrix(_api_manager->m_hmvp,(D3DXMATRIX*)&mat ));

	application_throw_hr(_api_manager->m_d3ddevice->SetVertexDeclaration(_api_manager->m_flat_declaration));

	application_throw_hr(_api_manager->m_d3ddevice->SetStreamSource(0,m_buffer, 0, sizeof(_vec3)));

	/* draw ground plane quads *********************************************************************/
	int p = 0;
	int bx = int(pos.x);
	int bz = int(pos.z);
	application_throw_hr(_fx->SetValue(_api_manager->m_hmaterial, &(material), sizeof(_material)));
	_vec3 * v = 0;
	application_throw_hr(m_buffer->Lock(0, 0, (void**)&v, 0));
	for (int x = -20; x <= 20; x++) {
		for (int z = -20; z <= 20; z++) {
			/*  1       4 */
			/** O*******O */
			/** * *     * */
			/** *   *   * */
			/** *     * * */
			/** O* *****O */
			/*  3       2 */
			v[p++]  = _vec3(bx+x-0.1f, 0, bz+z+0.1f);/*1*/
			v[p++]  = _vec3(bx+x+0.1f, 0, bz+z-0.1f);/*2*/
			v[p++]  = _vec3(bx+x+0.1f, 0, bz+z+0.1f);/*3*/
			v[p++]  = _vec3(bx+x-0.1f, 0, bz+z+0.1f);/*1*/
			v[p++]  = _vec3(bx+x-0.1f, 0, bz+z-0.1f);/*4*/
			v[p++]  = _vec3(bx+x+0.1f, 0, bz+z-0.1f);/*2*/

		}
	}
	application_throw_hr(m_buffer->Unlock()); 
	application_throw_hr(_fx->Begin(NULL, 0));
	application_throw_hr(_fx->BeginPass(0));
	application_throw_hr(_api_manager->m_d3ddevice->DrawPrimitive(D3DPT_TRIANGLELIST, 0, p/3));
	application_throw_hr(_fx->EndPass());
	application_throw_hr(_fx->End()); 
	/***********************************************************************************************/

	/* shader: object technique */
	application_error_hr(_fx->SetTechnique(_api_manager->m_htech) );

	_mat4 transform = m_aircraft.gettransform();

	// Fuselage
	_mat4 world = _scale(_vec3(2.0f, 0.8f, 1.0f))*_translate( _vec3(-0.5f, 0, 0) )*transform;
	_application->m_object_manager->drawcube(world,_vec4(0.6f,0.6f,0.6f,1.0f));

	// Rear Fuselage
	world = _scale(_vec3(2.0f, 0.5f, 0.5f))*_translate( _vec3(1.5f, 0.15f, 0) )*transform;
	_application->m_object_manager->drawcube(world,_vec4(0.6f,0.6f,0.6f,1.0f));

	// Wings
	world = _scale(_vec3(0.8f, 0.1f, 6.0f))*_translate( _vec3(0, 0.3f, 0) )*transform;
	_application->m_object_manager->drawcube(world,_vec4(0.6f,0.6f,0.6f,1.0f));

	// Rudder
	world = _scale(_vec3(0.75f, 1.15f, 0.1f))*_translate( _vec3(2.0f, 0.775f, 0) )*transform;
	_application->m_object_manager->drawcube(world,_vec4(0.6f,0.6f,0.6f,1.0f));

	// Tail-plane
	world = _scale(_vec3(0.85f, 0.1f, 2.0f))*_translate( _vec3(1.9f, 0, 0) )*transform;
	_application->m_object_manager->drawcube(world,_vec4(0.6f,0.6f,0.6f,1.0f));


	return true;
}

void flight_simulator::key( uint8_t key) {

	switch(key)
	{
	case 'q': case 'Q':
		m_rudder_control += 0.1f;
		break;

	case 'e': case 'E':
		m_rudder_control -= 0.1f;
		break;

	case 'w': case 'W':
		m_left_wing_control -= 0.1f;
		m_right_wing_control -= 0.1f;
		break;

	case 's': case 'S':
		m_left_wing_control += 0.1f;
		m_right_wing_control += 0.1f;
		break;

	case 'd': case 'D':
		m_left_wing_control -= 0.1f;
		m_right_wing_control += 0.1f;
		break;

	case 'a': case 'A':
		m_left_wing_control += 0.1f;
		m_right_wing_control -= 0.1f;
		break;

	case 'x': case 'X':
		m_left_wing_control = 0.0f;
		m_right_wing_control = 0.0f;
		m_rudder_control = 0.0f;
		break;

	case 'r': case 'R':
		resetplane();
		break;

	}

	// make sure the controls are in range
	if (m_left_wing_control < -1.0f) { m_left_wing_control = -1.0f; }
	else if (m_left_wing_control > 1.0f) { m_left_wing_control = 1.0f; }
	if (m_right_wing_control < -1.0f) { m_right_wing_control = -1.0f; }
	else if (m_right_wing_control > 1.0f) { m_right_wing_control = 1.0f; }
	if (m_rudder_control < -1.0f) { m_rudder_control = -1.0f; }
	else if (m_rudder_control > 1.0f) { m_rudder_control = 1.0f; }

	// update the control surfaces
	m_left_wing.setcontrol(m_left_wing_control);
	m_right_wing.setcontrol(m_right_wing_control);
	m_rudder.setcontrol(m_rudder_control);

}