Shader "Custom/NewUnlitShader"
{
	Properties{
		_MainTex("Albedo", 2D) = "white" { }
		// normal map texture on the material,
		// default to dummy "flat surface" normalmap
		_BumpMap("Normal Map", 2D) = "bump" {}

		_TangentVector("Tangent vector", Vector) = (0, 1, 0, 0)
		_BitangentVector("Bitangent vector", Vector) = (0, 0, 1, 0)
	}
	SubShader
	{
		Pass
		{
			CGPROGRAM
			#pragma vertex vert
			#pragma fragment frag
			// include file that contains UnityObjectTom helper function
			#include "UnityCG.cginc"

			// vertex input: position, normal
			struct appdata {
				float4 vertex : POSITION;
				float3 normal : NORMAL;
				float4 tangent : TANGENT;
				float2 uv : TEXCOORD;
			};

			struct v2f {
				float3 worldPos : TEXCOORD0;
				// these three vectors will hold a 3x3 rotation matrix
				// that transforms from tangent to world space
				half3 tspace0 : TEXCOORD1; // tangent.x, bitangent.x, normal.x
				half3 tspace1 : TEXCOORD2; // tangent.y, bitangent.y, normal.y
				half3 tspace2 : TEXCOORD3; // tangent.z, bitangent.z, normal.z
				// texture coordinate for the normal map
				float2 uv : TEXCOORD4;
				float3 normal : TEXCOORD5;

				float4 pos : SV_POSITION;
			};


			float4 _MainTex_ST;
			

			// vertex shader: takes object space normal as input too
			v2f vert(appdata v)
			{
				v2f o;
				o.pos = UnityObjectToClipPos(v.vertex);
				o.worldPos = mul(unity_ObjectToWorld, v.vertex).xyz;
				half3 wNormal = UnityObjectToWorldNormal(v.normal);
				half3 wTangent = UnityObjectToWorldDir(v.tangent.xyz);
				// compute bitangent from cross product of normal and tangent
				half tangentSign = v.tangent.w * unity_WorldTransformParams.w;
				half3 wBitangent = cross(wNormal, wTangent) * tangentSign;
				// output the tangent space matrix
				o.tspace0 = half3(wTangent.x, wBitangent.x, wNormal.x);
				o.tspace1 = half3(wTangent.y, wBitangent.y, wNormal.y);
				o.tspace2 = half3(wTangent.z, wBitangent.z, wNormal.z);
				o.uv = TRANSFORM_TEX(v.uv, _MainTex);
				o.normal = wNormal;
				return o;
			}

			sampler2D _BumpMap;
			half4 _TangentVector;
			half4 _BitangentVector;

			fixed4 frag(v2f i) : SV_Target
			{
				
				// sample the normal map, and decode from the Unity encoding
				half3 tnormal = UnpackNormal(tex2D(_BumpMap, i.uv));

				// micro normal vector
				half3 m;
				m.x = dot(i.tspace0, tnormal);
				m.y = dot(i.tspace1, tnormal);
				m.z = dot(i.tspace2, tnormal);

				half3 d1 = normalize(_TangentVector.xyz);
				half3 d2 = normalize(_BitangentVector.xyz);
				half3 d3 = -d1;
				half3 d4 = -d2;

				half theta = acos(dot(i.normal, m));
				half P = 2 * UNITY_INV_PI * tan(theta);

				half3 m_prime = m - dot(m, i.normal) * i.normal;
				m_prime = normalize(m_prime);

				half S1 = max(0, dot(m_prime, d1));
				half S2 = max(0, dot(m_prime, d2));
				half S3 = max(0, dot(m_prime, d3));
				half S4 = max(0, dot(m_prime, d4));

				fixed4 c = 0;
				c.r = S1 * P;
				c.g = S2 * P;
				c.b = S3 * P;
				c.a = S4 * P;
				return c;
			}
			ENDCG
		}
	}
}
