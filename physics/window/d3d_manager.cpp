#include "d3d_manager.h"

#include "application.h"
#include "d3d_window.h"

d3d_manager* d3d_manager::_manager = NULL;

d3d_manager::d3d_manager(){


	application_zero(&m_d3dpp,sizeof(m_d3dpp));

	m_d3dobject = NULL;
	m_d3ddevice = NULL;


	m_flat_declaration   = NULL;
	m_vertex_declaration = NULL;

	application_zero(&m_light,sizeof(_light));

	m_fx         = NULL;

	m_hmvp       = (D3DXHANDLE)NULL;
	m_htech      = (D3DXHANDLE)NULL;
	m_hworld     = (D3DXHANDLE)NULL;
	m_hlight     = (D3DXHANDLE)NULL;
	m_hmaterial  = (D3DXHANDLE)NULL;
	m_hflat_tech = (D3DXHANDLE)NULL;

}

bool d3d_manager::init(){

	// step 1: create the idirect3d9 object.
	m_d3dobject = Direct3DCreate9(D3D_SDK_VERSION);
	if( !m_d3dobject ) { application_throw("d3dobject"); }

	// step 2: verify hardware support for specified formats in windowed and full screen modes.
	D3DDISPLAYMODE mode;
	HRESULT HR = m_d3dobject->GetAdapterDisplayMode(D3DADAPTER_DEFAULT, &mode);
	if(HR != D3D_OK){ application_throw("hr"); }

	application_throw_hr(m_d3dobject->CheckDeviceType(D3DADAPTER_DEFAULT, D3DDEVTYPE_HAL, mode.Format, mode.Format, true));
	application_throw_hr(m_d3dobject->CheckDeviceType(D3DADAPTER_DEFAULT, D3DDEVTYPE_HAL, D3DFMT_X8R8G8B8, D3DFMT_X8R8G8B8, false));

	// step 3: check for requested vertex processing and pure device.
	D3DCAPS9 caps;
	application_throw_hr(m_d3dobject->GetDeviceCaps(D3DADAPTER_DEFAULT, D3DDEVTYPE_HAL, &caps));

	DWORD devbehaviorflags = 0;
	if( caps.DevCaps & D3DDEVCAPS_HWTRANSFORMANDLIGHT ) { devbehaviorflags |= D3DCREATE_HARDWARE_VERTEXPROCESSING; }
	else { devbehaviorflags |= D3DCREATE_SOFTWARE_VERTEXPROCESSING; }

	// if pure device and hw t&l supported
	if( caps.DevCaps & D3DDEVCAPS_PUREDEVICE && devbehaviorflags & D3DCREATE_HARDWARE_VERTEXPROCESSING) { devbehaviorflags |= D3DCREATE_PUREDEVICE; }

	// step 4: fill out the D3DPRESENT_PARAMETERS structure.
	m_d3dpp.BackBufferWidth            = 0;
	m_d3dpp.BackBufferHeight           = 0;
	m_d3dpp.BackBufferFormat           = D3DFMT_UNKNOWN;
	m_d3dpp.BackBufferCount            = 1;
	m_d3dpp.MultiSampleType            = D3DMULTISAMPLE_NONE;
	m_d3dpp.MultiSampleQuality         = 0;
	m_d3dpp.SwapEffect                 = D3DSWAPEFFECT_DISCARD;
	m_d3dpp.hDeviceWindow              = _window->m_hwnd;
	m_d3dpp.Windowed                   = true;
	m_d3dpp.EnableAutoDepthStencil     = true;
	m_d3dpp.AutoDepthStencilFormat     = D3DFMT_D24S8;
	m_d3dpp.Flags                      = 0;
	m_d3dpp.FullScreen_RefreshRateInHz = D3DPRESENT_RATE_DEFAULT;
	m_d3dpp.PresentationInterval       = D3DPRESENT_INTERVAL_IMMEDIATE;

	// create the device.
	application_throw_hr(m_d3dobject->CreateDevice(

		D3DADAPTER_DEFAULT,// primary adapter
		D3DDEVTYPE_HAL,    // device type
		_window->m_hwnd,   // window associated with device
		devbehaviorflags,  // vertex processing
		&m_d3dpp,          // present parameters
		&m_d3ddevice       // return created device
		));

	// checking shader version 2.1 or greater required
	application_zero( &caps , sizeof(D3DCAPS9) );
	application_throw_hr(m_d3ddevice->GetDeviceCaps(&caps));
	if( caps.VertexShaderVersion < D3DVS_VERSION(2, 0) ) { application_throw("dev caps"); }
	if( caps.PixelShaderVersion  < D3DPS_VERSION(2, 0) ) { application_throw("dev caps"); }

	/* line_declaration ********************************************************/
	D3DVERTEXELEMENT9 lineelements[] = {
		{0, 0,  D3DDECLTYPE_FLOAT3, D3DDECLMETHOD_DEFAULT, D3DDECLUSAGE_POSITION, 0},
		D3DDECL_END()
	};
	application_throw_hr(_api_manager->m_d3ddevice->CreateVertexDeclaration(lineelements, &m_flat_declaration));
	/*****************************************************************************/

	/* vertex_declaration ********************************************************/
	D3DVERTEXELEMENT9 vertexelements[] = {
		{0, 0,  D3DDECLTYPE_FLOAT3, D3DDECLMETHOD_DEFAULT, D3DDECLUSAGE_POSITION, 0},
		{0, 12, D3DDECLTYPE_FLOAT3, D3DDECLMETHOD_DEFAULT, D3DDECLUSAGE_NORMAL, 0},
		D3DDECL_END()
	};
	application_throw_hr(_api_manager->m_d3ddevice->CreateVertexDeclaration(vertexelements, &m_vertex_declaration));
	/*****************************************************************************/

	return buildfx();
}

void d3d_manager::clear(){
	application_releasecom(m_d3dobject);
	application_releasecom(m_d3ddevice);
	application_releasecom(m_fx);
	application_releasecom(m_vertex_declaration);
}

bool d3d_manager::reset(){
	_application->onlostdevice();
	application_throw_hr(m_d3ddevice->Reset( &m_d3dpp) );
	_application->onresetdevice();
	return true;
}

bool d3d_manager::buildfx() {

	_string shader = 
		"	struct light"
		"{"
		"	float4 ambient;"
		"	float4 diffuse;"
		"	float3 dirW;  "
		"};"

		"struct material"
		"{"
		"	float4 ambient;"
		"	float4 diffuse;"
		"};"

		"uniform extern light    g_light;"
		"uniform extern material g_material;"

		"uniform extern float4x4 g_mvp; "
		"uniform extern float4x4 g_world;" 


		"uniform extern texture g_tex;"
		"sampler tex_s = sampler_state {"
		"     Texture = <g_tex>;"
		" };"

		"struct OutputVS{  "
		"     float4 pos      : POSITION0; "
		"     float3 normal   : TEXCOORD1;"
		"}; "

		"OutputVS VertexShader_ ( float3 pos : POSITION0, float3 normal : NORMAL0 ) {"
		"         OutputVS outVS = (OutputVS)0;"
		"         outVS.pos    = mul( float4(pos,1.0f)    , g_mvp);"
		"		 outVS.normal = mul( float4(normal,0.0f) , g_world).xyz;"
		"         return outVS;"
		"	}"

		"float4 PixelShader_ ( float3 normal:TEXCOORD1 ) : COLOR { "


		"	 float s = max(dot( -g_light.dirW, normal), 0.0f);"
		"	 s = min(s, 1.0f);"

		"    float3  ambient = g_light.ambient * g_material.ambient;"
		"     float3  diffuse = g_light.diffuse * g_material.diffuse;"
		"	 diffuse = diffuse *s;"

		"     return float4((  (ambient + diffuse) ) , g_material.diffuse.a); "
		"} "

		"technique object_tech { "
		"     pass P0 "
		"         { "
		"             vertexShader = compile vs_2_0 VertexShader_(); "
		"             pixelShader  = compile ps_2_0 PixelShader_();"
		"         }"
		"}"

		"struct OutputVS_flat{ float4 pos      : POSITION0; }; "
		"OutputVS_flat VertexShader_flat ( float3 pos : POSITION0) {"
		"         OutputVS_flat outVS = (OutputVS_flat)0;"
		"		 outVS.pos    = mul( float4(pos,1.0f)    , g_mvp);"
		"         return outVS;"
		"	}"

		"float4 PixelShader_flat () : COLOR { return g_material.diffuse;  } "

		"technique flat_tech { "
		"     pass P0 "
		"         { "
		"             vertexShader = compile vs_2_0 VertexShader_flat(); "
		"             pixelShader  = compile ps_2_0 PixelShader_flat();"
		"         }"
		"}";

	/* load effect **********************************************************/
	ID3DXBuffer* errors = 0;
	HRESULT fx_result =D3DXCreateEffect(_api_manager->m_d3ddevice,
		shader.m_data,
		shader.m_count,
		0, 0, D3DXSHADER_DEBUG, 0, &m_fx, &errors);
	if( errors )    { application_throw((char*)errors->GetBufferPointer()); }
	if( fx_result ) { application_throw("effect file"); }
	/*****************************************************************************/

	/* obtain handles. **********************************************************/
	m_hmvp       = m_fx->GetParameterByName(0, "g_mvp");
	m_htech      = m_fx->GetTechniqueByName("object_tech");
	m_hworld     = m_fx->GetParameterByName(0, "g_world");
	m_hlight     = m_fx->GetParameterByName(0, "g_light");
	m_hmaterial  = m_fx->GetParameterByName(0, "g_material");
	m_hflat_tech = m_fx->GetTechniqueByName("flat_tech");


	if( m_hmvp       == (D3DXHANDLE)NULL ){ application_throw("mvp handle"); }
	if( m_htech      == (D3DXHANDLE)NULL ){ application_throw("technique handle"); }
	if( m_hworld     == (D3DXHANDLE)NULL ){ application_throw("world handle"); }
	if( m_hlight     == (D3DXHANDLE)NULL ){ application_throw("light handle"); }
	if( m_hmaterial  == (D3DXHANDLE)NULL ){ application_throw("material handle"); }	
	if( m_hflat_tech == (D3DXHANDLE)NULL ){ application_throw("technique handle"); }
	/****************************************************************************/

	/* single scene light  ***************************/
	m_light.ambient   = _vec4(0.98f,0.98f,0.98f,1.0f);
	m_light.diffuse   = _vec4(0.4f,0.4f,0.4f,1.0f);
	m_light.direction = _vec3(1.0f,0.0f,0.0f);
	application_throw_hr(m_fx->SetValue(m_hlight, &m_light, sizeof(_light)));
	/***********************************************************************/

	return true;
}
