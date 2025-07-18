#pragma once

#include "application_header.h"

struct pascal_object;

struct object_manager;

struct application : public application_object {

    application();

	/*application_object interface functions*/
	bool init();
	void run();
	void clear();
	bool update();
	/****************************************/

	/*d3dx device state functions */
    void onlostdevice();
    void onresetdevice();
	/*******************************/

	/* holds the current demo class*/
	application_object * m_demo;

    ID3DXFont* m_font;

	/***** object manager*********/
    object_manager* m_object_manager;

	/*camera******************/
    _vec3      m_up;
    _vec3      m_look;
    _vec3      m_target;
    _vec3      m_position;
    _mat4      m_view;
    _mat4      m_projection;

    float      m_y_pos;
    float      m_x_pos;

	float      m_yaw;
    float      m_pitch;
    float      m_yaw_pos;
    float      m_pitch_pos;
    float      m_distance;
	/*************************/

	/*cursor*****************/
	float      m_x_cursor_pos;
    float      m_y_cursor_pos;
	/************************/


	/*application global static variables *********************/
    static application *  _instance;
    static HINSTANCE      _win32_instance;
	/**********************************************************/

	/***generates a _mesh from a ._mesh resource file **************/
	static bool loadmeshfile(const LPVOID data,_mesh* submeshes,bool bones=true);
	/**********************************************************/
	
	static LPVOID  s_lpdata;
	static HGLOBAL s_hglobal;
	static HRSRC   s_hresource;
	static LPVOID  getresourcedata(int id);
	static void    freeresourcedata();

	/*clear color*/
	static D3DCOLOR _clear_color;

};
