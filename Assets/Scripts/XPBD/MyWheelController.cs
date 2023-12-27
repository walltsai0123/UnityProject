using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

namespace XPBD
{
    public class MyWheelController : MonoBehaviour
    {
        [SerializeField] Wheel frontLeft;
        [SerializeField] Wheel frontRight;
        [SerializeField] Wheel rearLeft;
        [SerializeField] Wheel rearRight;
        [SerializeField] Rigid rack;

        private Rigid carBody;
        public float accelration = 500f;
        public float brakingForce = 300f;
        public float maxTurnAngle = 15f;

        private float currentAccel = 0f;
        private float currentBrakeForce = 0f;
        private float currentTurnAngle = 0f;

        private Vector3 axis = Vector3.right;

        private Hinge fLHinge, fRHinge;

        private void Awake()
        {
            carBody = GetComponent<Rigid>();

            Hinge[] hinges;
            hinges = rack.GetComponents<Hinge>();
            Debug.Assert(hinges.Length == 2);
            fLHinge = hinges[0];
            fRHinge = hinges[1];
        }
        // Update is called once per frame
        void Update()
        {
            currentAccel = accelration * Input.GetAxis("Vertical");
            currentTurnAngle = maxTurnAngle * Input.GetAxis("Horizontal");

            if (Input.GetKey(KeyCode.Space))
                currentBrakeForce = brakingForce;
            else
                currentBrakeForce = 0f;

            float3 Axis = math.rotate(carBody.Rotation, axis);
            frontLeft.RotateAxis = Axis;
            frontRight.RotateAxis = Axis;
            rearLeft.RotateAxis = Axis;
            rearRight.RotateAxis = Axis;

            //frontLeft.MotorTorque = currentAccel;
            //frontRight.MotorTorque = currentAccel;
            rearLeft.MotorTorque = currentAccel;
            rearRight.MotorTorque = currentAccel;

            frontLeft.BrakeTorque = currentBrakeForce;
            frontRight.BrakeTorque = currentBrakeForce;
            rearLeft.BrakeTorque = currentBrakeForce;
            rearRight.BrakeTorque = currentBrakeForce;

            frontLeft.SteerAngle = currentTurnAngle;
            frontRight.SteerAngle = currentTurnAngle;

            //rack.fext = currentTurnAngle * 100f * Axis;

            fLHinge.targetAngle = currentTurnAngle;
            fRHinge.targetAngle = currentTurnAngle;
        }
    }

}
