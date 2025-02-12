Shader "Custom/HeightToNormal"
{
    Properties
    {
        _MainTex ("Texture", 2D) = "white" {}
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            sampler2D _MainTex;
            float4 _MainTex_TexelSize;
            float _Strength;

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

            v2f vert(appdata v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.uv = v.uv;
                return o;
            }

            float3 CalculateNormal(float2 uv)
            {
                // Sample heights around the current pixel
                float hL = tex2D(_MainTex, uv + float2(-_MainTex_TexelSize.x, 0)).r; // Left
                float hR = tex2D(_MainTex, uv + float2(_MainTex_TexelSize.x, 0)).r;  // Right
                float hD = tex2D(_MainTex, uv + float2(0, -_MainTex_TexelSize.y)).r; // Down
                float hU = tex2D(_MainTex, uv + float2(0, _MainTex_TexelSize.y)).r;  // Up

                // Calculate gradient
                float3 normal;
                normal.x = _Strength * (hL - hR);
                normal.y = _Strength * (hD - hU);
                normal.z = 1;

                // Normalize the normal
                return normalize(normal);
            }

            fixed4 frag(v2f i) : SV_Target
            {
                float3 normal = CalculateNormal(i.uv);
                return fixed4(normal * 0.5 + 0.5, 1.0);
            }
            ENDCG
        }
    }
}
