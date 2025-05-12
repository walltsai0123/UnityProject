using UnityEngine;
using Unity.Mathematics;

#if USE_FLOAT
using REAL = System.Single;
using REAL2 = Unity.Mathematics.float2;
using REAL3 = Unity.Mathematics.float3;
using REAL4 = Unity.Mathematics.float4;
using REAL2x2 = Unity.Mathematics.float2x2;
using REAL3x3 = Unity.Mathematics.float3x3;
using REAL3x4 = Unity.Mathematics.float3x4;
#else
using REAL = System.Double;
using REAL2 = Unity.Mathematics.double2;
using REAL3 = Unity.Mathematics.double3;
using REAL4 = Unity.Mathematics.double4;
using REAL2x2 = Unity.Mathematics.double2x2;
using REAL3x3 = Unity.Mathematics.double3x3;
using REAL3x4 = Unity.Mathematics.double3x4;
#endif

namespace XPBD
{
    public class Wheel : MonoBehaviour
    {
        [SerializeField]
        private Rigid rim;
        [SerializeField]
        private SoftBody rubber;
        public float MotorTorque { get; set; } = 0f;

        private float brakeTorque = 0f;
        public float BrakeTorque
        {
            get => brakeTorque;
            set => brakeTorque = Mathf.Max(0f, value);
        }
        public float SteerAngle { get; set; } = 0f;

        public float3 RotateAxis { get; set; } = float3.zero;

        readonly float3 axis = Vector3.right;
        private void Update()
        {
            rim.Tau += MotorTorque * math.rotate(rim.Rotation, axis);

            // Apply brake torque
            //if (math.length(rim.omega) > Util.EPSILON)
            //{
            //    // Check angular velocity direction
            //    float dir = (math.dot(rim.omega, RotateAxis)) < 0 ? 1f : -1f;

            //    rim.Tau += dir * brakeTorque * RotateAxis;
            //}

            // Apply brake to veloctiy
            if (math.length(rim.omega) > Util.EPSILON)
            {
                // Get angular velocity and rotate axis
                REAL vel = math.length(rim.omega);
                REAL3 axis = math.normalize(rim.omega);

                // Get final angular veloctity
                REAL final_vel = vel - brakeTorque;
                if (final_vel < Util.EPSILON)
                    final_vel = 0f;

                rim.omega = final_vel * axis;
            }

        }
    }
}

