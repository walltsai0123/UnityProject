#if !defined(MY_SHADOWS_INCLUDED)
#define MY_SHADOWS_INCLUDED

#include "UnityCG.cginc"

#define _HeightMap _ParallaxMap
	#define _Height _Parallax

#define TERRAIN_USE_SEPARATE_VERTEX_SAMPLER
#if defined(TERRAIN_USE_SEPARATE_VERTEX_SAMPLER)
        Texture2D _ParallaxMap;
        Texture2D _TerrainNormalMap;
        SamplerState sampler__TerrainNormalMap;
        SamplerState vertex_linear_clamp_sampler;
    #else
        sampler2D _ParallaxMap;
        sampler2D _TerrainNormalMap;
#endif

float4 _Color;
sampler2D _MainTex;
float4 _MainTex_ST;
float _Cutoff;

//sampler2D _ParallaxMap; 
float _Parallax;

struct VertexInput {
	float4 vertex : POSITION;
	float3 normal : NORMAL;
	float2 uv : TEXCOORD0;
};

struct VertexOutput {
	float4 position : SV_POSITION;
	#if SHADOWS_NEED_UV
		float2 uv : TEXCOORD0;
	#endif
	#if defined(SHADOWS_CUBE)
		float3 lightVec : TEXCOORD1;
	#endif
};

#define vert vertShadowCaster
VertexOutput vertShadowCaster (VertexInput v) {
	VertexOutput i;
	UNITY_INITIALIZE_OUTPUT(VertexOutput, i);


	float2 samplerCoords = v.uv;
	#if defined(TERRAIN_USE_SEPARATE_VERTEX_SAMPLER)
    	float displacement = UnpackHeightmap(_HeightMap.SampleLevel(vertex_linear_clamp_sampler, samplerCoords, 0));
		float3 geomNormal = UnpackNormal(_TerrainNormalMap.SampleLevel(sampler__TerrainNormalMap, samplerCoords, 0));
    #else
    	float displacement = UnpackHeightmap(tex2Dlod(_HeightMap, float4(samplerCoords, 0, 0)));
		float3 geomNormal = UnpackNormal(tex2D(_TerrainNormalMap, samplerCoords));
	#endif
	displacement = (displacement - 0.0) * _Height;
	v.vertex.xyz += (float3(0,1,0)) * displacement;
    
	
	#if defined(SHADOWS_CUBE)
		i.position = UnityObjectToClipPos(v.vertex);
		i.lightVec =
			mul(unity_ObjectToWorld, v.vertex).xyz - _LightPositionRange.xyz;
	#else
		i.position = UnityClipSpaceShadowCasterPos(v.vertex.xyz, v.normal);
		i.position = UnityApplyLinearShadowBias(i.position);
	#endif
	return i;
}

float4 fragShadowCaster (VertexOutput i) : SV_TARGET {
	#if defined(SHADOWS_CUBE)
		float depth = length(i.lightVec) + unity_LightShadowBias.x;
		depth *= _LightPositionRange.w;
		return UnityEncodeCubeShadowDepth(depth);
	#else
		return 0;
	#endif
}
#endif