Shader "Custom/GaussianFilter"
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
            #include "UnityCG.cginc"

            sampler2D _MainTex;
            float4 _MainTex_TexelSize;

            float3 ApplyGaussianFilter(float2 uv)
            {
                float3 color = 0.0;

                // 定義 3×3 高斯核
                float kernel[3][3] = {
                    { 1, 2, 1 },
                    { 2, 4, 2 },
                    { 1, 2, 1 }
                };

                // 遍歷 3×3 區域並加權求和
                for (int y = -1; y <= 1; y++)
                {
                    for (int x = -1; x <= 1; x++)
                    {
                        float2 offset = float2(x, y) * _MainTex_TexelSize.xy;
                        float weight = kernel[y + 1][x + 1] / 16.0; // 歸一化
                        color += tex2D(_MainTex, uv + offset).rgb * weight;
                    }
                }
                return color;
            }

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

            fixed4 frag(v2f i) : SV_Target
            {
                float3 filteredColor = ApplyGaussianFilter(i.uv);
                return fixed4(filteredColor, 1.0);
            }
            ENDCG
        }
    }
}
