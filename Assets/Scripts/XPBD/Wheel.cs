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
            if (math.length(rim.omega) > math.EPSILON)
            {
                // Check angular velocity direction
                float dir = (math.dot(rim.omega, RotateAxis)) < 0 ? 1f : -1f;

                rim.Tau += dir * brakeTorque * RotateAxis;
            }

        }
    }
}

