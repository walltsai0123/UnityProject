using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

namespace XPBD
{
    public class MyWheelController : MonoBehaviour
    {
        enum DriveTrain
        {
            FWD,
            RWD,
            AWD
        }
        [SerializeField] DriveTrain drive;
        [SerializeField] Wheel frontLeft;
        [SerializeField] Wheel frontRight;
        [SerializeField] Wheel rearLeft;
        [SerializeField] Wheel rearRight;

        [SerializeField] Hinge fLArm;
        [SerializeField] Hinge fRArm;

        [SerializeField] Rigid carBody;
        public float accelration = 500f;
        public float brakingForce = 300f;
        public float maxTurnAngle = 15f;

        [SerializeField]
        private float currentAccel = 0f;
        private float currentBrakeForce = 0f;
        private float currentTurnAngle = 0f;

        private Vector3 axis = Vector3.right;

        public float simulatedVertical = 0f;
        public int startFrame = 600;
        public int durationFrames = 600;
        public int turnStartFrame = 600;
        public int turnDurationFrame = 600;
        public int currentFrame = 0;
        public bool isAutoMode = false;
        private void Awake()
        {
            carBody = GetComponent<Rigid>();
        }
        void Update()
        {
            float simulatedHorizontal = 0;
            // ¼ÒÀÀ Vertical Input ¬° 1¡A«ùÄò 600 frame
            if (currentFrame > startFrame && currentFrame <= startFrame + durationFrames)
            {
                simulatedVertical = 1f;
                
            }
            else
            {
                simulatedVertical = 0f;
            }
            if(currentFrame > turnStartFrame && currentFrame <= turnStartFrame + turnDurationFrame && transform.position.z > 135)
            {
                simulatedHorizontal = -1;
                simulatedVertical *= 0.75f;
            }
            currentFrame++;

            float vertical = 0;
            float horizontal = 0;
            if (isAutoMode)
            {
                vertical = simulatedVertical;
                horizontal = simulatedHorizontal;
            }
            else
            {
                vertical = Input.GetAxis("Vertical");
                horizontal = Input.GetAxis("Horizontal");
            }
            //float vertical = isAutoMode ? simulatedVertical : Input.GetAxis("Vertical");
            currentAccel = accelration * vertical;
            currentTurnAngle = maxTurnAngle * horizontal;

            if (Input.GetKey(KeyCode.Space))
                currentBrakeForce = brakingForce;
            else
                currentBrakeForce = 0f;

            float3 Axis = math.rotate(carBody.Rotation, axis);
            frontLeft.RotateAxis = Axis;
            frontRight.RotateAxis = Axis;
            rearLeft.RotateAxis = Axis;
            rearRight.RotateAxis = Axis;

            switch(drive)
            {
                case DriveTrain.FWD:
                    frontLeft.MotorTorque = currentAccel / 2;
                    frontRight.MotorTorque = currentAccel / 2;
                    break;
                case DriveTrain.RWD:
                    rearLeft.MotorTorque = currentAccel / 2;
                    rearRight.MotorTorque = currentAccel / 2;
                    break;
                case DriveTrain.AWD:
                    var accel = currentAccel / 4;
                    frontLeft.MotorTorque = accel;
                    frontRight.MotorTorque = accel;
                    rearLeft.MotorTorque = accel;
                    rearRight.MotorTorque = accel;
                    break;
            }

            frontLeft.BrakeTorque = currentBrakeForce;
            frontRight.BrakeTorque = currentBrakeForce;
            rearLeft.BrakeTorque = currentBrakeForce;
            rearRight.BrakeTorque = currentBrakeForce;

            frontLeft.SteerAngle = currentTurnAngle;
            frontRight.SteerAngle = currentTurnAngle;


            fLArm.targetOn = true;
            fRArm.targetOn = true;
            fLArm.targetAngle = currentTurnAngle;
            fRArm.targetAngle = currentTurnAngle;
        }
        private void OnDrawGizmos()
        {
            //if(!Application.isPlaying)
            //    return;
            //Gizmos.color = Color.black;
            //
            //Gizmos.DrawRay((float3)carBody.Position, (float3)carBody.vel);
        }
    }

}
