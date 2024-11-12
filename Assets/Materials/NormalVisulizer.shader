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

            fixed4 frag(v2f i) : SV_Target
            {
                float4 normals = tex2D(_CameraDepthNormalsTexture, i.uv);
                return 0.5 * normals + 0.5;
            }
            ENDCG
        }
    }
}
