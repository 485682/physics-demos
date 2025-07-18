#include "sailing_simulator.h"


#include "clock.h"
#include "application.h"
#include "d3d_manager.h"
#include "object_manager.h"



sailing_simulator::sailing_simulator():
	m_sail(_mat3(0.0f,0,0, 0,0,0, 0,0,-1.0f), _vec3(2.0f, 0.0f, 0.0f), &m_wind_speed),
	m_buoyancy(_vec3(0.0f, 0.5f, 0.0f), 1.0f, 3.0f, 1.6f),
	m_sail_control(0), m_wind_speed(0,0,0){  }


bool sailing_simulator::init(){

	/*ground plane quads ************************************************/
	m_buffer = NULL;
	application_throw_hr(_api_manager->m_d3ddevice->CreateVertexBuffer(
		11000 * sizeof(_vec3),
		D3DUSAGE_WRITEONLY,0, D3DPOOL_MANAGED,	
		&m_buffer, 0));
	if(!m_buffer){ application_throw("line buffer"); }
	/********************************************************************/

    // set up the boat's rigid body *************************************/
    m_sail_boat.setposition(0, 1.6f, 0);
    m_sail_boat.setorientation(1,0,0,0);

    m_sail_boat.setvelocity(0,0,0);
    m_sail_boat.setrotation(0,0,0);

    m_sail_boat.setmass(200.0f);
    _mat3 it;
    it.setblockinertiatensor(_vec3(2,1,1), 100.0f);
    m_sail_boat.setinertiatensor(it);

    m_sail_boat.setdamping(0.8f, 0.8f);

    m_sail_boat.setacceleration( _utility::gravity );
    m_sail_boat.calculatederiveddata();

    m_sail_boat.setawake();
    m_sail_boat.setcansleep(false);
	/********************************************************************/

    /* add force generators to registry *****/
    m_registry.add(&m_sail_boat, &m_sail);
    m_registry.add(&m_sail_boat, &m_buoyancy);
	/****************************************/

	return true;
}
void sailing_simulator::clear(){
	application_releasecom(m_buffer);
}
bool sailing_simulator::update(){

	// find the duration of the last frame in seconds
    float duration = application_clock->m_last_frame_seconds;
	if (duration > 0.0f){

		// Start with no forces or acceleration.
		m_sail_boat.clearaccumulators();

		// add the forces acting on the boat.
		m_registry.updateforces(duration);

		// update the boat's physics.
		m_sail_boat.integrate(duration);

		// change the wind speed.
		m_wind_speed = m_wind_speed * 0.9f + r.randomxzvector(1.0f);

	}

	return render();
}

bool sailing_simulator::render(){

	_vec3 pos = m_sail_boat.getposition();
	_vec3 offset(4.0f, 0, 0);
	offset = m_sail_boat.gettransform().transformdirection(offset);

	//set camera view matrix
	_application->m_view =_lookatrh( 
		_vec3(pos.x+offset.x, pos.y+10.0f, pos.z+offset.z),
		_vec3(pos.x, pos.y, pos.z), 
		_utility::up);

	_material material;
	material.diffuse = _vec4(0.6f,0.6f,0.6f,1.0f);

	// shader: flat ( quads ) technique
	application_error_hr(_fx->SetTechnique(_api_manager->m_hflat_tech) );
	_mat4 mat = _application->m_view*_application->m_projection;

	application_throw_hr(_fx->SetMatrix(_api_manager->m_hmvp,(D3DXMATRIX*)&mat ));

	application_throw_hr(_api_manager->m_d3ddevice->SetVertexDeclaration(_api_manager->m_flat_declaration));

	application_throw_hr(_api_manager->m_d3ddevice->SetStreamSource(0,m_buffer, 0, sizeof(_vec3)));

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

	// shader: object technique
	application_error_hr(_fx->SetTechnique(_api_manager->m_htech) );

	_mat4 transform = m_sail_boat.gettransform();

	// Left Hull
	_mat4 world = _scale(_vec3(2.0f, 0.4f, 0.4f))*_translate( _vec3(0, 0, -1.0f) )*transform;
	_application->m_object_manager->drawcube(world,_vec4(0.4f,0.4f,0.4f,1.0f));
	// Right Hull
	world = _scale(_vec3(2.0f, 0.4f, 0.4f))*_translate( _vec3(0, 0, 1.0f) )*transform;
	_application->m_object_manager->drawcube(world,_vec4(0.4f,0.4f,0.4f,1.0f));
	// Deck
	world = _scale(_vec3(1.0f, 0.1f, 2.0f))*_translate( _vec3(0, 0.3f, 0) )*transform;
	_application->m_object_manager->drawcube(world,_vec4(0.4f,0.4f,0.4f,1.0f));
	// Mast
	world = _scale(_vec3(0.1f, 3.0f, 0.1f))*_translate( _vec3(0, 1.8f, 0) )*transform;
	_application->m_object_manager->drawcube(world,_vec4(0.4f,0.4f,0.4f,1.0f));

	return true;
}
