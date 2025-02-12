#ifndef UNITY_HEIGHTMAPBASE_INCLUDED
#define UNITY_HEIGHTMAPBASE_INCLUDED

#include "UnityPBSLighting.cginc"
#include "AutoLight.cginc"

#define TESSELLATION_TANGENT 1
#define TESSELLATION_UV1 1
#define TESSELLATION_UV2 1
#define VERTEX_DISPLACEMENT 1

#define _HeightMap _ParallaxMap
#define _Height _Parallax

half4 _Color;

sampler2D _MainTex;
float4 _MainTex_ST;

sampler2D _MetallicMap;
float _Metallic;
float _Smoothness;

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

sampler2D _BumpMap;
float _BumpScale;

//sampler2D _ParallaxMap;
float _Parallax;

struct VertexInput{
    float4 vertex : POSITION;
    float3 normal : NORMAL;
    float4 tangent : TANGENT;
    float2 uv : TEXCOORD0;
    float2 uv1 : TEXCOORD1;
    float2 uv2 : TEXCOORD2;
};

struct VertexOutput {
    float4 pos : SV_POSITION;
    float3 worldPos : TEXCOORD0;

    //3x3 rotation matrix that transforms from tangent to world space
    half3 tspace[3] : TEXCOORD1;

    #if defined(VERTEXLIGHT_ON)
		float3 vertexLightColor : TEXCOORD3;
	#endif

    // texture coordinate for the normal map
    float4 uv : TEXCOORD4;

    SHADOW_COORDS(5)
};

void ComputeVertexLightColor (inout VertexOutput i) {
    #if defined(VERTEXLIGHT_ON)
        i.vertexLightColor = Shade4PointLights(
            unity_4LightPosX0, unity_4LightPosY0, unity_4LightPosZ0,
            unity_LightColor[0].rgb, unity_LightColor[1].rgb,
            unity_LightColor[2].rgb, unity_LightColor[3].rgb,
            unity_4LightAtten0, i.worldPos.xyz, i.normal
        );
    #endif
}

VertexOutput vert (VertexInput v)
{
    VertexOutput i;
    UNITY_INITIALIZE_OUTPUT(VertexOutput, i);

    float2 samplerCoords = v.uv;
    i.uv.xy = TRANSFORM_TEX(v.uv, _MainTex);
    i.uv.wz = v.uv;

    #if defined(TERRAIN_USE_SEPARATE_VERTEX_SAMPLER)
        float displacement = UnpackHeightmap(_HeightMap.SampleLevel(vertex_linear_clamp_sampler, samplerCoords, 0));
    #else
        float displacement = UnpackHeightmap(tex2Dlod(_HeightMap, float4(samplerCoords, 0, 0)));
    #endif

    displacement = (displacement - 0.0) * _Height;
    v.normal = normalize(v.normal);
	v.vertex.xyz += (float3(0,1,0)) * displacement;

    #if defined(TERRAIN_USE_SEPARATE_VERTEX_SAMPLER)
        float3 geomNormal = UnpackScaleNormal(_TerrainNormalMap.SampleLevel(sampler__TerrainNormalMap, samplerCoords, 0), 1);
    #else
        float3 geomNormal = UnpackScaleNormal(tex2Dlod(_TerrainNormalMap, float4(samplerCoords, 0, 0)), 1); 
    #endif

    geomNormal = geomNormal.xzy;

    i.pos = UnityObjectToClipPos(v.vertex);
    i.worldPos = mul(unity_ObjectToWorld, v.vertex).xyz;
    half3 wNormal = UnityObjectToWorldNormal(v.normal);
    half3 wTangent = UnityObjectToWorldDir(v.tangent.xyz);
    
    half tangentSign = v.tangent.w * unity_WorldTransformParams.w;
    half3 wBitangent = cross(wNormal, wTangent) * tangentSign;
    half3x3 tangentToWorld = half3x3(wTangent, wBitangent, wNormal);
    i.tspace[0] = tangentToWorld[0];
    i.tspace[1] = tangentToWorld[1];
    i.tspace[2] = tangentToWorld[2];

    
    float3 geomTangent = normalize(cross(geomNormal, float3(0, 0, 1)));
    float3 geomBitangent = normalize(cross(geomTangent, geomNormal));
    i.tspace[0] = geomTangent;
    i.tspace[1] = geomBitangent;
    i.tspace[2] =UnityObjectToWorldNormal(geomNormal);

    TRANSFER_SHADOW(i);
    ComputeVertexLightColor(i);

    return i;
}

UnityLight CreateLight (VertexOutput i) {
	UnityLight light;
	//light.dir = _WorldSpaceLightPos0.xyz;
	light.color = _LightColor0.rgb;

    #if defined(POINT) || defined(POINT_COOKIE) || defined(SPOT)
            light.dir = normalize(_WorldSpaceLightPos0.xyz - i.worldPos.xyz);
        #else
            light.dir = _WorldSpaceLightPos0.xyz;
    #endif

    UNITY_LIGHT_ATTENUATION(attenuation, i, i.worldPos.xyz);
    light.color = _LightColor0.rgb * attenuation;	
    return light;
}

UnityIndirect CreateIndirectLight (VertexOutput i) {
	UnityIndirect indirectLight;
	indirectLight.diffuse = 0;
	indirectLight.specular = 0;

	#if defined(VERTEXLIGHT_ON)
		indirectLight.diffuse = i.vertexLightColor;
	#endif

    indirectLight.diffuse += max(0, ShadeSH9(float4(i.tspace[2], 1)));
	return indirectLight;
}

fixed4 frag (VertexOutput i) : SV_Target
{
    // sample the normal map, and decode from the Unity encoding
    half3 normalTangent = UnpackScaleNormal(tex2D(_BumpMap, i.uv), _BumpScale);
    half3 worldNormal;

    half3 tangent = i.tspace[0].xyz;
    half3 binormal = i.tspace[1].xyz;
    half3 normal = i.tspace[2].xyz;
    worldNormal = tangent * normalTangent.x + binormal * normalTangent.y + normal * normalTangent.z;

    float3 viewDir = normalize(_WorldSpaceCameraPos - i.worldPos);
    half3 albedo = _Color.rgb * tex2D (_MainTex, i.uv.xy).rgb;

    float3 specularTint;
	float oneMinusReflectivity;
	albedo = DiffuseAndSpecularFromMetallic(
		albedo, _Metallic, specularTint, oneMinusReflectivity
	);

    //return float4(worldNormal, 1);
	return UNITY_BRDF_PBS(
		albedo, specularTint,
		oneMinusReflectivity, _Smoothness,
		worldNormal, viewDir,
		CreateLight(i), CreateIndirectLight(i)
	);
}

#endif //UNITY_HEIGHTMAPBASE_INCLUDED