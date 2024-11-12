Shader "Custom/CustomTerrainShader"{
    Properties
    {
        [HideInInspector] _MainTex ("BaseMap (RGB)", 2D) = "white" {}
        [HideInInspector] _Color ("Main Color", Color) = (1,1,1,1)
        [HideInInspector] _TerrainHolesTexture("Holes Map (RGB)", 2D) = "white" {}

        _TangentVector("Tangent vector", Vector) = (1, 0, 0, 0)
        _BitangentVector("Bitangent vector", Vector) = (0, 0, 1, 0)
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
            #pragma multi_compile_instancing
            //#pragma multi_compile_fog
            #pragma target 3.0
            #pragma instancing_options assumeuniformscaling nomatrices nolightprobe nolightmap forwardadd

            #include "UnityCG.cginc"
            #include "Lighting.cginc"

            #pragma multi_compile_local __ _NORMALMAP

            #define TERRAIN_INSTANCED_PERPIXEL_NORMAL
            #include "TerrainSplatmapCommon.cginc" 

            struct v2f
            {
                float4 vertex : SV_POSITION;
                float4 uv : TEXCOORD0;
                float3 normal : TEXCOORD1;

                UNITY_VERTEX_INPUT_INSTANCE_ID
            };

            sampler2D _MainTex;
            float4 _MainTex_ST;
            v2f vert (appdata_full v)
            {
                v2f o;

                UNITY_SETUP_INSTANCE_ID(v);
                UNITY_TRANSFER_INSTANCE_ID(v, o); 
                
                Input data;
                SplatmapVert(v, data);
                                
                o.uv = data.tc;
                o.vertex = UnityObjectToClipPos(v.vertex);
                // get normal from camera front
                o.normal = normalize(float3(UNITY_MATRIX_V._m20, UNITY_MATRIX_V._m21, UNITY_MATRIX_V._m22));
                return o;
            }

            half3 _TangentVector;
            half3 _BitangentVector;

            fixed4 frag (v2f i) : SV_Target
            {
                // sample the texture
                Input data;
                data.tc = i.uv;
                half4 splat_control;
                half weight;
                fixed4 mixedDiffuse;
                fixed3 temp = fixed3(0,1,0);
                SplatmapMix(data, splat_control, weight, mixedDiffuse, temp);

                half3 m = temp.xzy;


                half3 d1 = normalize(_TangentVector.xyz);
                half3 d2 = normalize(_BitangentVector.xyz);
                half3 d3 = -d1;
                half3 d4 = -d2;
                
                half theta = acos(dot(i.normal, m));
                float P = 2.0 * UNITY_INV_PI * tan(theta);

                half3 m_prime = m - dot(m, i.normal) * i.normal;
                m_prime = normalize(m_prime);
                
                half S1 = max(0, dot(m_prime, d1));
                half S2 = max(0, dot(m_prime, d2));
                half S3 = max(0, dot(m_prime, d3));
                half S4 = max(0, dot(m_prime, d4));

                fixed4 final_color = 0;
                final_color.r = S1 * P;
                final_color.g = S2 * P;
                final_color.b = S3 * P;
                final_color.a = S4 * P;

                fixed3 dir_color =  temp * 0.5 + 0.5;
                //final_color = fixed4(dir_color, 1);
                return final_color;
            }
            ENDCG
        }
    }
}