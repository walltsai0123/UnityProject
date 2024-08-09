Shader "Custom/TerrainWithNormalOutput" {
    Properties {
        [HideInInspector] _MainTex ("BaseMap (RGB)", 2D) = "white" {}
        [HideInInspector] _Color ("Main Color", Color) = (1,1,1,1)

        [HideInInspector] _TerrainHolesTexture("Holes Map (RGB)", 2D) = "white" {}

        _TangentVector("Tangent vector", Vector) = (1, 0, 0, 0)
        _BitangentVector("Bitangent vector", Vector) = (0, 0, 1, 0)
        _NormalVector("Normal vector", Vector) = (0, 1, 0, 0)
    }

    SubShader {
        Tags {
            "Queue" = "Geometry-100"
            "RenderType" = "Opaque"
            "TerrainCompatible" = "True"
        }

        CGPROGRAM
        #pragma surface surf Standard vertex:SplatmapVert 
        //finalcolor:SplatmapFinalColor finalgbuffer:SplatmapFinalGBuffer addshadow fullforwardshadows
        #pragma instancing_options assumeuniformscaling nomatrices nolightprobe nolightmap forwardadd
        //#pragma multi_compile_fog // needed because finalcolor oppresses fog code generation.
        #pragma target 3.0
        #include "UnityPBSLighting.cginc"

        #pragma multi_compile_local_fragment __ _ALPHATEST_ON
        #pragma multi_compile_local __ _NORMALMAP
         
        #define TERRAIN_STANDARD_SHADER
        #define TERRAIN_INSTANCED_PERPIXEL_NORMAL
        #define TERRAIN_SURFACE_OUTPUT SurfaceOutputStandard
        #include "TerrainSplatmapCommon.cginc" 

        half _Metallic0;
        half _Metallic1;
        half _Metallic2;
        half _Metallic3;

        half _Smoothness0;
        half _Smoothness1;
        half _Smoothness2;
        half _Smoothness3;

        half3 _TangentVector;
        half3 _BitangentVector;
        half3 _NormalVector;

        void surf (Input IN, inout SurfaceOutputStandard o) {
            half4 splat_control;
            half weight;
            fixed4 mixedDiffuse;
            half4 defaultSmoothness = half4(_Smoothness0, _Smoothness1, _Smoothness2, _Smoothness3);
            SplatmapMix(IN, defaultSmoothness, splat_control, weight, mixedDiffuse, o.Normal);
            o.Albedo = mixedDiffuse.rgb;
            o.Alpha = weight;
            o.Smoothness = mixedDiffuse.a;
            o.Metallic = dot(splat_control, half4(_Metallic0, _Metallic1, _Metallic2, _Metallic3));

            o.Normal = o.Normal.zxy;
        }

        fixed4 LightingNoLighting(SurfaceOutputStandard s, fixed3 lightDir, fixed atten)
        {
            fixed4 c;
            c.rgb = s.Albedo; 
            c.a = s.Alpha;
            return c;
        }
        ENDCG

        UsePass "Hidden/Nature/Terrain/Utilities/PICKING"
        UsePass "Hidden/Nature/Terrain/Utilities/SELECTION"
    }

    Dependency "AddPassShader"    = "Hidden/TerrainEngine/Splatmap/Standard-AddPass"
    Dependency "BaseMapShader"    = "Hidden/TerrainEngine/Splatmap/Standard-Base"
    Dependency "BaseMapGenShader" = "Hidden/TerrainEngine/Splatmap/Standard-BaseGen"
    
    Fallback "Nature/Terrain/Diffuse"
}

//fixed4 LightingNoLighting(SurfaceOutput s, fixed3 lightDir, fixed atten)
//        {
//            fixed4 c;
//            c.rgb = s.Albedo; 
//            c.a = s.Alpha;
//            return c;
//        }
// ploughing friction model
            //half3 d1 = normalize(_TangentVector.xyz);
            //half3 d2 = normalize(_BitangentVector.xyz);
            //half3 d3 = -d1;
            //half3 d4 = -d2;
            
            //half theta = acos(dot(o.Normal, _NormalVector));
            //float P = 2.0 * UNITY_INV_PI * tan(theta);
            
            //half3 m_prime = _NormalVector - dot(_NormalVector, o.Normal) * o.Normal;
            //m_prime = normalize(m_prime);
            
            //half S1 = max(0, dot(m_prime, d1));
            //half S2 = max(0, dot(m_prime, d2));
            //half S3 = max(0, dot(m_prime, d3));
            //half S4 = max(0, dot(m_prime, d4));

            //o.Albedo.r = S1 * P;
            //o.Albedo.g = S2 * P;
            //o.Albedo.b = S3 * P;
            //o.Alpha = S4 * P;