#include "object_manager.h"

#include "application.h"

#include "d3d_window.h"
#include "d3d_manager.h"

object_manager* object_manager::_manager = NULL;


bool object_manager::readmesh( _mesh * mesh,const char* file,bool back){

	if(!application::readrtmeshfile(file,mesh) ){ application_throw("readmesh"); }

	/* get vertex buffer pointer */
	application_throw_hr(_api_manager->m_d3ddevice->CreateVertexBuffer(
		mesh->m_submeshes[0].m_vertices.m_count * sizeof(_vertex),
		D3DUSAGE_WRITEONLY,0, D3DPOOL_MANAGED,	
		&(mesh->m_submeshes[0].m_vertex_buffer), 0));

	if(!mesh->m_submeshes[0].m_vertex_buffer){ application_throw("vertex buffer"); }

	_vertex * v = 0;
	/* lock vertex buffer and write */
	application_throw_hr(mesh->m_submeshes[0].m_vertex_buffer->Lock(0, 0, (void**)&v, 0));
	for(uint32_t ii=0;ii<mesh->m_submeshes[0].m_vertices.m_count;ii++){
		v[ii] = mesh->m_submeshes[0].m_vertices[ii];
		//st to uv************************
		v[ii].m_uv.y = 1.0f-v[ii].m_uv.y;
		//********************************
	}
	application_throw_hr(mesh->m_submeshes[0].m_vertex_buffer->Unlock());

	application_throw_hr(_api_manager->m_d3ddevice->CreateIndexBuffer(
		mesh->m_submeshes[0].m_indices.m_count * sizeof(WORD),
		D3DUSAGE_WRITEONLY,D3DFMT_INDEX16,
		D3DPOOL_MANAGED, &(mesh->m_submeshes[0].m_index_buffer), 0));
	if(!mesh->m_submeshes[0].m_index_buffer){ application_throw("index_buffer"); }

	WORD* indices = 0;
	application_throw_hr(mesh->m_submeshes[0].m_index_buffer->Lock(0, 0, (void**)&(indices), 0));
	for(uint32_t ii=0;ii<mesh->m_submeshes[0].m_indices.m_count/3;ii++){

		uint32_t pos = ii*3;
		if(back){
			indices[pos  ] = WORD(mesh->m_submeshes[0].m_indices[pos]);
			indices[pos+1] = WORD(mesh->m_submeshes[0].m_indices[pos+1]);
			indices[pos+2] = WORD(mesh->m_submeshes[0].m_indices[pos+2]);
		}else{
			//* conversion from right hand( opengl ) to left hand( direct3d ) Coordinate Systems
			//* requires clockwise rotation of triangles
			/*https://learn.microsoft.com/en-us/windows/win32/direct3d9/coordinate-systems*/
			indices[pos  ] = WORD(mesh->m_submeshes[0].m_indices[pos]);
			indices[pos+1] = WORD(mesh->m_submeshes[0].m_indices[pos+2]);
			indices[pos+2] = WORD(mesh->m_submeshes[0].m_indices[pos+1]);
		}
	}
	application_throw_hr(mesh->m_submeshes[0].m_index_buffer->Unlock());

	return true;
}

bool object_manager::init(){


	if(!readmesh( &m_cube_mesh   , "cube._mesh"   )){ return false; }
	if(!readmesh( &m_sphere_mesh , "sphere._mesh" )){ return false; }
	if(!readmesh( &m_mirror_cube_mesh   , "cube._mesh"   ,true)){ return false; }
	if(!readmesh( &m_mirror_sphere_mesh , "sphere._mesh" ,true)){ return false; }
	return true;
}
void object_manager::clear(){

	application_releasecom(m_cube_mesh.m_submeshes[0].m_index_buffer);
	application_releasecom(m_cube_mesh.m_submeshes[0].m_vertex_buffer);
	application_releasecom(m_sphere_mesh.m_submeshes[0].m_index_buffer);
	application_releasecom(m_sphere_mesh.m_submeshes[0].m_vertex_buffer);

}

bool object_manager::drawcube(const _mat4 & world,const _vec4 & color,bool back){

	_material material;
	material.diffuse   = color;
	material.ambient   = color*_vec4(0.6f,0.6f,0.6f,1.0f);

	_mat4 mat = world*_application->m_view*_application->m_projection;

	application_throw_hr(_fx->SetMatrix(_api_manager->m_hmvp,(D3DXMATRIX*)&mat ));
	application_throw_hr(_fx->SetMatrix(_api_manager->m_hworld, (D3DXMATRIX*)&world));

	application_throw_hr(_api_manager->m_d3ddevice->SetVertexDeclaration(_api_manager->m_vertex_declaration));

	if(back){
		application_throw_hr(_api_manager->m_d3ddevice->SetStreamSource(0, m_mirror_cube_mesh.m_submeshes[0].m_vertex_buffer, 0, sizeof(_vertex)));
		application_throw_hr(_api_manager->m_d3ddevice->SetIndices(m_mirror_cube_mesh.m_submeshes[0].m_index_buffer));
	}else{
		application_throw_hr(_api_manager->m_d3ddevice->SetStreamSource(0, m_cube_mesh.m_submeshes[0].m_vertex_buffer, 0, sizeof(_vertex)));
		application_throw_hr(_api_manager->m_d3ddevice->SetIndices(m_cube_mesh.m_submeshes[0].m_index_buffer));
	}

	application_throw_hr(_fx->SetValue(_api_manager->m_hmaterial, &(material), sizeof(_material)));

	application_throw_hr(_fx->Begin(NULL, 0));
	application_throw_hr(_fx->BeginPass(0));
	application_throw_hr(_api_manager->m_d3ddevice->DrawIndexedPrimitive(
		D3DPT_TRIANGLELIST, 0, 0,
		m_cube_mesh.m_submeshes[0].m_vertices.m_count, 0,
		m_cube_mesh.m_submeshes[0].m_indices.m_count/3));
	application_throw_hr(_fx->EndPass());
	application_throw_hr(_fx->End()); 

	return true; 

}
bool object_manager::drawsphere(const _mat4 & world,const _vec4 & color,bool back){

	_material material;
	material.diffuse   = color;
	material.ambient   = color*_vec4(0.6f,0.6f,0.6f,1.0f);

	_mat4 mat = world*_application->m_view*_application->m_projection;

	application_throw_hr(_fx->SetMatrix(_api_manager->m_hmvp,(D3DXMATRIX*)&mat ));
	application_throw_hr(_fx->SetMatrix(_api_manager->m_hworld, (D3DXMATRIX*)&world));

	application_throw_hr(_api_manager->m_d3ddevice->SetVertexDeclaration(_api_manager->m_vertex_declaration));
	if(back){
		application_throw_hr(_api_manager->m_d3ddevice->SetStreamSource(0, m_mirror_sphere_mesh.m_submeshes[0].m_vertex_buffer, 0, sizeof(_vertex)));
		application_throw_hr(_api_manager->m_d3ddevice->SetIndices(m_mirror_sphere_mesh.m_submeshes[0].m_index_buffer));
	}else{
		application_throw_hr(_api_manager->m_d3ddevice->SetStreamSource(0, m_sphere_mesh.m_submeshes[0].m_vertex_buffer, 0, sizeof(_vertex)));
		application_throw_hr(_api_manager->m_d3ddevice->SetIndices(m_sphere_mesh.m_submeshes[0].m_index_buffer));
	}
	application_throw_hr(_fx->SetValue(_api_manager->m_hmaterial, &(material), sizeof(_material)));

	application_throw_hr(_fx->Begin(NULL, 0));
	application_throw_hr(_fx->BeginPass(0));
	application_throw_hr(_api_manager->m_d3ddevice->DrawIndexedPrimitive(
		D3DPT_TRIANGLELIST, 0, 0,
		m_sphere_mesh.m_submeshes[0].m_vertices.m_count, 0,
		m_sphere_mesh.m_submeshes[0].m_indices.m_count/3));
	application_throw_hr(_fx->EndPass());
	application_throw_hr(_fx->End());

	return true;

}
