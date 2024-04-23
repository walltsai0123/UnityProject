Shader "CmdBuffer/PureColorShader"
{
    Properties
    {
        _MainColor("Main Color",Color) = (1,1,1,1)
        _MainAlpha("Main Alpha",Range(0,1)) = 0.3
    }
    SubShader
    {
        Tags { "RenderType"="Transparent" "Queue"="Transparent" }
        LOD 100
 
        Pass
        { 
            Blend SrcAlpha OneMinusSrcAlpha
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
 
            #include "UnityCG.cginc"
 
            struct appdata
            {
                float4 vertex : POSITION;
            };
 
            struct v2f
            {
                float4 vertex : SV_POSITION;
            };
 
            float4 _MainColor;
            float _MainAlpha;
 
            v2f vert (appdata v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                return o;
            }
 
            fixed4 frag (v2f i) : SV_Target
            {
                fixed4 col = _MainColor;
                col.a = _MainAlpha;
                return col;
            }
            ENDCG
        }
    }
}