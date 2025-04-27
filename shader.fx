
struct light
{
	float4 ambient;
	float4 diffuse;
	float3 dirW;  
};

struct material
{
	float4 ambient;
	float4 diffuse;
};

uniform extern light    g_light;
uniform extern material g_material;

uniform extern float4x4 g_mvp; 
uniform extern float4x4 g_world; 


uniform extern texture g_tex;
sampler tex_s = sampler_state {
     Texture = <g_tex>;
 };
 
struct OutputVS{  
     float4 pos      : POSITION0; 
     float3 normal   : TEXCOORD1;
}; 

OutputVS VertexShader_ ( float3 pos : POSITION0, float3 normal : NORMAL0 ) {
         OutputVS outVS = (OutputVS)0;
         outVS.pos    = mul( float4(pos,1.0f)    , g_mvp);
		 outVS.normal = mul( float4(normal,0.0f) , g_world).xyz;
         return outVS;
	}

float4 PixelShader_ ( float3 normal:TEXCOORD1 ) : COLOR { 


	 float s = max(dot( -g_light.dirW, normal), 0.0f);
	 s = min(s, 1.0f);

     float3  ambient = g_light.ambient * g_material.ambient;
     float3  diffuse = g_light.diffuse * g_material.diffuse;
	 diffuse = diffuse *s;

     return float4((  (ambient + diffuse) ) , g_material.diffuse.a); 
} 

technique object_tech { 
     pass P0 
         { 
             vertexShader = compile vs_2_0 VertexShader_(); 
             pixelShader  = compile ps_2_0 PixelShader_();
         }
}

struct OutputVS_flat{ float4 pos      : POSITION0; }; 
OutputVS_flat VertexShader_flat ( float3 pos : POSITION0) {
         OutputVS_flat outVS = (OutputVS_flat)0;
		 outVS.pos    = mul( float4(pos,1.0f)    , g_mvp);
         return outVS;
	}

float4 PixelShader_flat () : COLOR { return g_material.diffuse;  } 

technique flat_tech { 
     pass P0 
         { 
             vertexShader = compile vs_2_0 VertexShader_flat(); 
             pixelShader  = compile ps_2_0 PixelShader_flat();
         }
}