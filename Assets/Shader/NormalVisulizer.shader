Shader "Custom/NormalVisualizer"
{
    Properties
    {
    }
    SubShader
    {
        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            struct v2f
            {
                float4 pos : SV_POSITION;
                float2 uv : TEXCOORD0;
            };

            v2f vert(appdata_base v)
            {
                v2f o;
                o.pos = UnityObjectToClipPos(v.vertex);
                o.uv = v.texcoord;
                return o;
            }

            sampler2D _CameraDepthNormalsTexture;
            sampler2D _CameraDepthTexture;

            fixed4 frag(v2f i) : SV_Target
            {
                float4 enc = tex2D(_CameraDepthNormalsTexture, i.uv);
                float depth;
                float3 normal;
                DecodeDepthNormal(enc, depth, normal);
                

                fixed4 depthCLR = fixed4(depth, depth, depth, 1);
                fixed4 normalCLR = fixed4(0.5 * normal + 0.5, 1);

                return depthCLR;
            }
            ENDCG
        }
    }
}
