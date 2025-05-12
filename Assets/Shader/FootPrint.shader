Shader "Custom/FootPrint"
{
    Properties
    {
        _MainTex ("Texture", 2D) = "white" {}
        _Control ("Texture", 2D) = "red" {}
    }
    SubShader
    {
        ZTest Always Cull Off ZWrite Off

        CGINCLUDE

            #include "UnityCG.cginc"
            #include "TerrainTool.cginc"

            sampler2D _MainTex;
            float4 _MainTex_TexelSize;      // 1/width, 1/height, width, height

            sampler2D _BrushTex;
            float4 _BrushParams;
            #define BRUSH_STRENGTH      (_BrushParams[0])
            #define BRUSH_TARGETHEIGHT  (_BrushParams[1])
            #define kMaxHeight          (32766.0f/65535.0f)

            float terrainHeight;

            sampler2D _Control;
            //float4 _Control_ST;
            //float4 _Control_TexelSize;

            float2 groundMaterial0; 
            float2 groundMaterial1;
            float2 groundMaterial2;
            float2 groundMaterial3;
            
            struct appdata_t {
                float4 vertex : POSITION;
                float2 pcUV : TEXCOORD0;
            };

            struct v2f {
                float4 vertex : SV_POSITION;
                float2 pcUV : TEXCOORD0;
            };

            v2f vert(appdata_t v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.pcUV = v.pcUV;
                return o;
            }

            float HeightMapToTerrainHeight(float input)
            {
                input *= terrainHeight;
                input /= kMaxHeight;
                return input;
            }

            float TerrainHeightToHeightMap(float input)
            {
                input /= terrainHeight;
                input *= kMaxHeight;
                return input;
            }

        ENDCG

        Pass
        {
            Name "Raise/Lower Heights"

            CGPROGRAM
            #pragma vertex vert
            #pragma fragment RaiseHeight

            float4 RaiseHeight(v2f i) : SV_Target
            {
                float2 brushUV = PaintContextUVToBrushUV(i.pcUV);
                float2 heightmapUV = PaintContextUVToHeightmapUV(i.pcUV);

                // out of bounds multiplier
                float oob = all(saturate(brushUV) == brushUV) ? 1.0 : 0.0;

                // Get terrainHeight
                float height = UnpackHeightmap(tex2D(_MainTex, heightmapUV));
                height = HeightMapToTerrainHeight(height);

                float4 brushShape = oob * (tex2D(_BrushTex, brushUV));
                float down = -brushShape.r * BRUSH_STRENGTH;
                float up = brushShape.g * BRUSH_STRENGTH;

                //float4 splat_control = tex2D(_Control, brushUV);
                //float weight = dot(splat_control, float4(1,1,1,1));
                //splat_control /= (weight + 1e-3f);
                //float youngModulus = 0;
                //youngModulus += splat_control.r * groundMaterial0.r;
                //youngModulus += splat_control.g * groundMaterial1.r;
                //youngModulus += splat_control.b * groundMaterial2.r;
                //youngModulus += splat_control.a * groundMaterial3.r;
                //float dL = down / (youngModulus + 1e-3f);
                //
                //float poissonRatio = 0;
                //poissonRatio += splat_control.r * groundMaterial0.g;
                //poissonRatio += splat_control.g * groundMaterial1.g;
                //poissonRatio += splat_control.b * groundMaterial2.g;
                //poissonRatio += splat_control.a * groundMaterial3.g;

                //float dL_inc = up * (2 * poissonRatio) / (youngModulus + 1e-3f);
                //float final = dL + dL_inc;
                float final = up + down;
                final *= (abs(final) < 1e-3f) ? 0 : 1;

                float normalizedHeight = TerrainHeightToHeightMap(height + final);
                return PackHeightmap(clamp(normalizedHeight, 0, kMaxHeight));
            }
            ENDCG
        }

        Pass    // 0 = Select one channel and copy it into R channel
        {
            Name "Get Terrain Layer Channel"

            BlendOp Max

            CGPROGRAM
            #pragma vertex vert
            #pragma fragment GetLayer
            float4 GetLayer(v2f i) : SV_Target
            {
                float4 layerWeights = tex2D(_MainTex, i.pcUV);
                return layerWeights;
            }
            ENDCG
        }
    }
}
