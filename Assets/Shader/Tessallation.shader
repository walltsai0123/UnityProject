Shader "Custom/Tessallation"
{
    Properties
    {
        _TessellationUniform ("Tessellation Uniform", Range(1, 64)) = 1
        _TessellationEdgeLength ("Tessellation Edge Length", Range(5, 100)) = 50

        _Cutoff ("Alpha Cutoff", Range(0, 1)) = 0.5

        _Color ("Color", Color) = (1, 1, 1, 1)
        _MainTex ("Albedo", 2D) = "white" {}

        [NoScaleOffset] _BumpMap ("Normals", 2D) = "bump" {}
        _BumpScale ("Bump Scale", Float) = 1

        [NoScaleOffset] _TerrainNormalMap ("TerrainNormals", 2D) = "bump" {}

        [NoScaleOffset] _MetallicMap ("Metallic", 2D) = "white" {}
        [Gamma] _Metallic ("Metallic", Range(0, 1)) = 0
        _Smoothness ("Smoothness", Range(0, 1)) = 0.1

        //[NoScaleOffset] _ParallaxMap ("Height Map", 2D) = "black" {}
        _Parallax ("Height Scale", Range(0, 200)) = 0

        [HideInInspector] _SrcBlend ("_SrcBlend", Float) = 1
        [HideInInspector] _DstBlend ("_DstBlend", Float) = 0
        [HideInInspector] _ZWrite ("_ZWrite", Float) = 1
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        LOD 100

        Pass {
            Tags {
                "LightMode" = "ForwardBase"
            }
            Blend [_SrcBlend] [_DstBlend]
            ZWrite [_ZWrite]

            CGPROGRAM
            #pragma target 5.0
			#pragma shader_feature _TESSELLATION_EDGE

            #pragma multi_compile _ VERTEXLIGHT_ON
            #pragma multi_compile_fwdbase

            #pragma vertex TessellationVertex
            #pragma hull MyHullProgram
			#pragma domain MyDomainProgram
            #pragma fragment frag

            #include "HeightMapBase.cginc"
            #include "MyTessellation.cginc"
            ENDCG
        }

        Pass {
			Tags {
				"LightMode" = "ForwardAdd"
			}
            
            Blend [_SrcBlend] One
            ZWrite Off

			CGPROGRAM

			#pragma target 5.0
			#pragma shader_feature _TESSELLATION_EDGE

            #pragma multi_compile_fwdadd_fullshadows

			#pragma vertex TessellationVertex
            #pragma hull MyHullProgram
			#pragma domain MyDomainProgram
            #pragma fragment frag

            #include "HeightMapBase.cginc"
            #include "MyTessellation.cginc"

			ENDCG
		}

        Pass {
            Tags {
                "LightMode" = "ShadowCaster"
            }

            CGPROGRAM

            #pragma target 5.0
			#pragma shader_feature _TESSELLATION_EDGE

            //#pragma shader_feature _ _RENDERING_CUTOUT _RENDERING_FADE _RENDERING_TRANSPARENT
            //#pragma shader_feature _SEMITRANSPARENT_SHADOWS
            //#pragma shader_feature _SMOOTHNESS_ALBEDO
            //#pragma shader_feature _PARALLAX_MAP
			//#pragma shader_feature _TESSELLATION_EDGE
            //
            //#pragma multi_compile _ LOD_FADE_CROSSFADE
            //
            #pragma multi_compile_shadowcaster
            //#pragma multi_compile_instancing
            //#pragma instancing_options lodfade force_same_maxcount_for_gl

            
            #pragma vertex TessellationVertex
            #pragma hull MyHullProgram
			#pragma domain MyDomainProgram
            #pragma fragment fragShadowCaster

            #include "My Shadows.cginc"
            #include "MyTessellation.cginc"

            ENDCG
        }
    }

    CustomEditor "TessellationShaderGUI"
}
