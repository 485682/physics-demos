#pragma once

#include "application_header.h"

struct object_manager : application_object { 

	bool readmesh( _mesh * mesh,const char* file,bool back=false);
	 
	virtual bool init();
	virtual void clear();
	virtual bool update(){ return true; }

	bool drawcube(const _mat4 & world,const _vec4 & color,bool back=false);
	bool drawsphere(const _mat4 & world,const _vec4 & color,bool back=false);

	_mesh m_cube_mesh;
	_mesh m_sphere_mesh;
	_mesh m_mirror_cube_mesh;
	_mesh m_mirror_sphere_mesh;

	static object_manager* _manager;

};