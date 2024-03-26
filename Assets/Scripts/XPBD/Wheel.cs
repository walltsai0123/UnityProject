using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

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

        private void Update()
        {
            //float3 rotateAxis = math.rotate(Transform.rotation, axis);

            rim.Tau += MotorTorque * RotateAxis;

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
                float vel = math.length(rim.omega);
                float3 axis = math.normalize(rim.omega);

                // Get final angular veloctity
                float final_vel = vel - brakeTorque;
                if (final_vel < Util.EPSILON)
                    final_vel = 0f;

                rim.omega = final_vel * axis;
            }

        }
    }
}

