#pragma once

#include "application_header.h"

struct d3d_manager{

    d3d_manager();
    
	bool init();
    void clear();

	/* reset the d3d device*/
    bool reset();
	/***********************/

	/*load .fx fle**********/
    bool buildfx();
	/***********************/


	/*Direct3D **********************************/
    D3DPRESENT_PARAMETERS m_d3dpp;
    IDirect3D9*           m_d3dobject;
    IDirect3DDevice9*     m_d3ddevice;
    IDirect3DVertexDeclaration9* m_flat_declaration;
    IDirect3DVertexDeclaration9* m_vertex_declaration;
	/************************************************/

	/* shader data **********************************/
	_light    m_light;
	/************************************************/

    ID3DXEffect* m_fx;
	/* effect handles */
    D3DXHANDLE   m_hmvp;
    D3DXHANDLE   m_htech;
    D3DXHANDLE   m_hworld;
    D3DXHANDLE   m_hlight;
    D3DXHANDLE   m_hmaterial;
	D3DXHANDLE   m_hflat_tech;
	/************************************************/

    static d3d_manager* _manager;

};
