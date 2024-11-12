// Upgrade NOTE: replaced 'mul(UNITY_MATRIX_MVP,*)' with 'UnityObjectToClipPos(*)'

Shader "Custom/CollisionArrow"
{
    Properties
    {
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        LOD 100

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma target 5.0
            #include "UnityCG.cginc"

            #include <Assets/Scripts/Compute/settings.hlsl>
            #include <Assets/Scripts/Compute/util.hlsl>

            struct Collision{
                uint valid;
                REAL3 q;
                REAL3 pos;
                REAL3 N;
                REAL3 T;
                REAL3 B;
                REAL3 fn;
                REAL vn_;
            };

            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
            };

            struct v2f
            {
                float2 uv : TEXCOORD0;
                float4 vertex : SV_POSITION;
            };

            StructuredBuffer<Collision> collisions;

            float4 vert(uint id : SV_VertexID) : SV_POSITION
            {
                uint index = id / 2;
                float3 collisionPos = collisions[index].q;
                float3 collisionNormal = collisions[index].N;
                
                float3 arrowPosition = (id % 2 == 0) ? collisionPos : collisionPos + collisionNormal;
            
                return UnityObjectToClipPos(float4(arrowPosition, 1.0)); 
            }

            fixed4 frag() : SV_Target
            {
                return fixed4(1.0, 0.0, 0.0, 1.0);
            }
            ENDCG
        }
    }
}
