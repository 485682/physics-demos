#include "application.h"

#include "clock.h"
#include "d3d_window.h"
#include "d3d_manager.h"
#include "object_manager.h"




application*  application::_instance       = NULL;
HINSTANCE     application::_win32_instance = NULL;

D3DCOLOR      application::_clear_color    = D3DCOLOR_XRGB(255, 255, 255);


const _vec3 _utility::gravity        = _vec3(0, -9.81f, 0);
const _vec3 _utility::high_gravity   = _vec3(0, -19.62f, 0);
const _vec3 _utility::up             = _vec3(0, 1, 0);
const _vec3 _utility::right          = _vec3(1, 0, 0);
const _vec3 _utility::out_of_screen  = _vec3(0, 0, 1);
const _vec3 _utility::x              = _vec3(0, 1, 0);
const _vec3 _utility::y              = _vec3(1, 0, 0);
const _vec3 _utility::z              = _vec3(0, 0, 1);

float _utility::sleepepsilon         = 0.3f;


application::application(){

	m_font           = NULL;
	_window          = NULL;
	_api_manager     = NULL;
	m_object_manager = NULL;
	m_demo           = NULL;

	m_look           = _vec3(0.0f,-1.0f,0.0f);
	m_target         = _vec3(0.0f,0.0f,0.0f);
	m_position       = _vec3(0.0f,30.0f,0.0f);

	m_y_pos          = 0.0f;
	m_x_pos          = 0.0f;

	m_yaw            = -45.0f;
	m_pitch          = -10.0f;
	m_yaw_pos        = -45.0f;
	m_pitch_pos      = -10.0f;
	m_distance       =  10.0f;

	m_x_cursor_pos   = 0.0f;
	m_y_cursor_pos   = 0.0f;

}

bool application::init(){

	/*win32 HINSTANCE */
	_win32_instance = GetModuleHandle(NULL);

	_window          = new d3d_window();
	_api_manager     = new d3d_manager();
	m_object_manager = new object_manager();

	if(!_window->init())            { return false; }
	if(!_api_manager->init())       { return false; }
	if(!m_object_manager->init())   { return false; }

	application_clock = new clock();
	application_clock->init();

	/* when this flag is removed, the application exits */
	addflags(application_running); 

	/***************************************/

	/*stat display font****************************************/
	D3DXFONT_DESC fontDesc;

	fontDesc.Height          = 12;
	fontDesc.Width           = 8;
	fontDesc.Weight          = FW_MEDIUM;
	fontDesc.MipLevels       = 0;
	fontDesc.CharSet         = DEFAULT_CHARSET;
	fontDesc.OutputPrecision = OUT_DEFAULT_PRECIS;
	fontDesc.Quality         = DEFAULT_QUALITY;
	fontDesc.PitchAndFamily  = DEFAULT_PITCH;
	/*crude solution to avoid compiler warning*****************/
	application_zero(fontDesc.FaceName,20);
	const char *fontname = "Times New Roman";
	for(int i=0;i<16;i++){ fontDesc.FaceName[i] = fontname[i]; }
	application_throw_hr(D3DXCreateFontIndirect(_api_manager->m_d3ddevice, &fontDesc, &m_font));
	/**********************************************************/

	return true;
}

void application::run(){

	/* to initialize projection matrix */
	onresetdevice();

	/* clear messege queue*/
	_window->update();

	float second = 1.0f;
	while( testflags(application_running) ) {

		/**d3d device test.  error exits application**/
		removeflags(application_lostdev);

		// get the state of the graphics device.
		HRESULT hr = _api_manager->m_d3ddevice->TestCooperativeLevel();

		if( hr == D3DERR_DEVICELOST ) {  

			// if the device is lost and cannot be reset yet then
			// sleep for a bit and we'll try again on the next 
			// message loop cycle.

			Sleep(20);
			addflags( application_lostdev );  

		}else if( hr == D3DERR_DRIVERINTERNALERROR ) {

			// internal driver error...exit

			addflags( application_deverror );
			application_error("d3d device error");

		}else if( hr == D3DERR_DEVICENOTRESET ) {

			// The device is lost but we can reset and restore it.

			_api_manager->reset();
			addflags( application_lostdev );
		}
		/*********************************************/

		/* clock update *******************************************/
		application_clock->update();
		/**********************************************************/

		/* increment second */
		second += application_clock->m_last_frame_seconds;

		/*stat string generation **************************************************/
		static _string state;
		if(second >=1.0f) {
			/* update per second*/

			state = _string("mspf: ")+_utility::floattostring(application_clock->m_last_frame_milliseconds,true);
			state = state + _string(" fps: ");
			state = state + _utility::floattostring(application_clock->m_fps,true);
			state = state + _string(" frames: ");
			state = state + _utility::inttostring(int(application_clock->m_frame));
			state = state + _string(" fullscreen: F vsync: V");
			second = 0.0f;
		}
		/**************************************************************************/

		if( !(testflags(application_lostdev))  && !(testflags(application_deverror)) && !(testflags(application_paused))  ) {
			/* application update (render) ******************************************/

			update();

			/* clear d3d, begin render */
			application_error_hr(_api_manager->m_d3ddevice->Clear(0, 0, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, application::_clear_color, 1.0f, 0));
			application_error_hr(_api_manager->m_d3ddevice->BeginScene());



			/* render state strings */
			RECT rect;
			D3DCOLOR text_color;
			if( application::_clear_color==D3DCOLOR_XRGB(0,0,0)) { 
				text_color=D3DCOLOR_XRGB(255, 255, 255);
			}else { text_color=D3DCOLOR_XRGB(0, 0, 0); }

			application_zero(&rect,sizeof(RECT));
			rect.bottom = 20;
			rect.left   = 10;
			rect.right  = state.m_count*10;
			m_font->DrawText(0,state.m_data, -1,&rect, DT_LEFT | DT_VCENTER, text_color);
			_string demo_state = m_demo->getstatestring();
			if(demo_state.m_count>0){
				rect.top    = 20;
				rect.bottom = 45;
				rect.left   = 10;
				rect.right  = demo_state.m_count*10;
				m_font->DrawText(0,demo_state.m_data, -1,&rect, DT_LEFT | DT_VCENTER, text_color);
			}
			/************************************************************************/



			/* demo/ main-application  update */
			m_demo->update();

			application_error_hr(_api_manager->m_d3ddevice->EndScene());
			application_error_hr(_api_manager->m_d3ddevice->Present(0, 0, 0, 0));
			/************************************************************************/
		}

		if( testflags(application_deverror) ) { removeflags(application_running); }
		else { _window->update(); }/* winpoc (input) */
	}

	/* deallocate .. exiting */
	m_demo->clear();
	clear();
	/**********************/
}

void application::clear(){

	application_releasecom(m_font);

	if( m_object_manager ) { m_object_manager->clear(); }
	if( _api_manager     ) { _api_manager->clear(); }

	if( _window )          { delete _window; _window = NULL; }
	if( _api_manager )     { delete _api_manager; _api_manager = NULL; }
	if( m_object_manager ) { delete m_object_manager; m_object_manager = NULL; }

}


bool application::update(){

	if( testflags( application_spherical_cam ) ){ 

		/** spherical camera*********************************/
		m_pitch = (m_pitch >  -10.0f ) ? -10.0f  : m_pitch;
		m_yaw   = (m_yaw   >  180.0f) ? 180.0f : m_yaw;
		m_pitch = (m_pitch < -90.0f ) ?-90.0f  : m_pitch;
		m_yaw   = (m_yaw   < -180.0f) ?-180.0f : m_yaw;

		_mat4  r = _yawpitchroll(float(_radians(m_yaw)),float(_radians(m_pitch)),0.0f);
		_vec4 t  = r*_vec4(0,0,m_distance,0.0f);
		_vec4 up = r*_vec4(
			_utility::up.x,
			_utility::up.y,
			_utility::up.z,0.0f);

		m_up = _vec3(up.x,up.y,up.z);
		m_position = m_target + _vec3(t.x,t.y,t.z);
		m_look = _normalize(m_target-m_position);

		m_view = _lookatrh(m_position, m_target, m_up);
		//***************************************************/
	}

	//*cursor update*************************************/
	POINT cursor_position;
	if (GetCursorPos(&cursor_position)) {
		m_x_cursor_pos = float(cursor_position.x);
		m_y_cursor_pos = float(cursor_position.y);
	}else{ application_error("cursor pos");}
	//***************************************************/

	return true;
}

void application::onlostdevice() {
	application_error_hr(_fx->OnLostDevice());
	application_error_hr(m_font->OnLostDevice());
}

void application::onresetdevice() {
	application_error_hr(_fx->OnResetDevice());
	application_error_hr(m_font->OnResetDevice());

	/*resize causes reset, so update projection matrix */
	float w = (float)_api_manager->m_d3dpp.BackBufferWidth;
	float h = (float)_api_manager->m_d3dpp.BackBufferHeight;
	m_projection = _perspectivefovrh( float(_radians(45.0f)), w,h, 0.01f, 1000.0f);
	/***************************************************/
}

bool application::readrtmeshfile(const char* path,_mesh * mesh){

	FILE * file = NULL;
	fopen_s(&file,path,"rb");
	if(!file){ application_throw("readrtmeshfile"); }

	char  header_[7];
	application_zero(header_,7);

	/* file marked with 6 bytes ascii string _mesh_ */
	if( fread(header_,1,6,file) != 6 ){ application_throw("fread"); }
	if(!application_scm(header_,"_mesh_")) { application_throw("not _mesh_ file"); }

	uint16_t submesh_count_ = 0;
	/* read 2 byte unsinged int ( submesh count )*/
	if( fread((&submesh_count_),2,1,file) != 1 ){ application_throw("fread"); }
	printf("smcount : %u \n",submesh_count_);

	for(uint32_t i=0;i<submesh_count_;i++){

		_submesh submesh_;

		uint32_t index_count_ = 0;
		/* read 4 byte unsinged int ( vertex indicies count ) */
		if( fread((&index_count_),4,1,file) != 1 ){ application_throw("fread"); }

		int32_t * indices = new int32_t[index_count_];
		/* read indices 4 bytes each  */
		if( fread(indices,4,index_count_,file) != index_count_ ) { application_throw("fread"); }
		for(uint32_t ii=0;ii<index_count_;ii++) { submesh_.m_indices.pushback(indices[ii],true); }

		uint32_t vertex_count_ = 0;
		/* read 4 byte unsinged int ( vertex count ) */
		if( fread((&vertex_count_),4,1,file) != 1 ){ application_throw("fread"); }

		_vertex * vertices = new _vertex[vertex_count_];
		/* read vertices  */
		if( fread(vertices,sizeof(_vertex),vertex_count_,file) != vertex_count_ ){ application_throw("fread"); }
		for(uint32_t ii=0;ii<vertex_count_;ii++){
			submesh_.m_vertices.pushback(vertices[ii],true);
		}
		mesh->m_submeshes.pushback(submesh_,true);
	}

	uint16_t bone_count = 0;
	/* read 2 byte unsinged int (bone transform count ) */
	if( fread((&bone_count),2,1,file) == 1 ){ 

		if(bone_count){

			printf("bone_count: %i \n",bone_count);
			mesh->m_bones.allocate(bone_count);
			/* read bone transforms  */
			if( fread( (&(mesh->m_bones[0])) ,sizeof(_mat4 ),bone_count,file) != bone_count ){ application_throw("fread"); }

			uint16_t keyframe_count = 0;
			/* read 2 byte unsinged int (animation keyframe count ) */
			if( fread( (&keyframe_count) ,2,1,file) != 1 ){ application_throw("fread"); }

			if( keyframe_count){
				printf("keyframes: %i \n",keyframe_count);
				/* read animation bone transforms  */
				for(uint32_t ii=0;ii<keyframe_count;ii++){
					_matrix_array keyframe;
					keyframe.allocate(bone_count);
					if( fread( &(keyframe[0]) ,sizeof(_mat4 ),bone_count,file) != bone_count ){ application_throw("fread"); }
					mesh->m_keyframes.pushback(keyframe);
				}
			}

		}
	}
	fclose(file);
	return true;
}

