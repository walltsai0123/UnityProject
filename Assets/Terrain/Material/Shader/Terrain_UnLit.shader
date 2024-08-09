Shader "Custom/Terrain_UnLit"
{
    Properties
    {
        [HideInInspector] _MainTex ("BaseMap (RGB)", 2D) = "white" {}
        [HideInInspector] _Color ("Main Color", Color) = (1,1,1,1)
        [HideInInspector] _TerrainHolesTexture("Holes Map (RGB)", 2D) = "white" {}
    }
    SubShader
    {
        Tags {
            "RenderType" = "Opaque"
            "TerrainCompatible" = "True"
        }
        LOD 100

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma instancing_options assumeuniformscaling nomatrices nolightprobe nolightmap forwardadd

            #include "UnityCG.cginc"
            #include "Lighting.cginc"

            #pragma multi_compile_local __ _NORMALMAP
            //#define TERRAIN_STANDARD_SHADER
            #define TERRAIN_INSTANCED_PERPIXEL_NORMAL
            #include "TerrainSplatmapCommon.cginc" 

            struct v2f
            {
                float4 uv : TEXCOORD0;
                float4 vertex : SV_POSITION;
                float3 normal : TEXCOORD1;
            };

            sampler2D _MainTex;
            float4 _MainTex_ST;

            v2f vert (appdata_full v)
            {
                v2f o;
                //o.vertex = UnityObjectToClipPos(v.vertex);
                //o.uv.xy = TRANSFORM_TEX(v.texcoord.xy, _MainTex);

                Input data;
                SplatmapVert(v, data);
                
                o.uv = data.tc;
                o.vertex = UnityObjectToClipPos(v.vertex);;
                return o;
            }

            fixed4 frag (v2f i) : SV_Target
            {
                // sample the texture
                Input data;
                data.tc = i.uv;
                half4 splat_control;
                half weight;
                fixed4 mixedDiffuse;
                SplatmapMix(data, splat_control, weight, mixedDiffuse, i.normal);


                return fixed4(i.normal,1);
            }
            ENDCG
        }
    }
}
