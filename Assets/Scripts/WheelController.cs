using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WheelController : MonoBehaviour
{
    public enum Drive
    {
        FrontWheel,
        RearWheel,
        FourWheel
    }
    [SerializeField] WheelCollider frontLeft;
    [SerializeField] WheelCollider frontRight;
    [SerializeField] WheelCollider rearLeft;
    [SerializeField] WheelCollider rearRight;

    [SerializeField] Transform frontLeftTransform;
    [SerializeField] Transform frontRightTransform;
    [SerializeField] Transform rearLeftTransform;
    [SerializeField] Transform rearRightTransform;

    public float accelration = 500f;
    public float brakingForce = 300f;
    public float maxTurnAngle = 15f;
    public Drive drive = Drive.FrontWheel;

    private float currentAccel = 0f;
    private float currentBrakeForce = 0f;
    private float currentTurnAngle = 0f;

    private void FixedUpdate()
    {
        
        currentAccel = accelration * Input.GetAxis("Vertical");
        currentTurnAngle = maxTurnAngle * Input.GetAxis("Horizontal");


        if (Input.GetKey(KeyCode.Space))
            currentBrakeForce = brakingForce;
        else
            currentBrakeForce = 0f;

        switch(drive)
        {
            case Drive.FrontWheel:
                frontLeft.motorTorque = currentAccel;
                frontRight.motorTorque = currentAccel;
                break;
            case Drive.RearWheel:
                rearLeft.motorTorque = currentAccel;
                rearRight.motorTorque = currentAccel;
                break;
            case Drive.FourWheel:
                frontLeft.motorTorque = currentAccel;
                frontRight.motorTorque = currentAccel;
                rearLeft.motorTorque = currentAccel;
                rearRight.motorTorque = currentAccel;
                break;
        }

        frontLeft.brakeTorque = currentBrakeForce;
        frontRight.brakeTorque = currentBrakeForce;
        rearLeft.brakeTorque = currentBrakeForce;
        rearRight.brakeTorque = currentBrakeForce;

        frontLeft.steerAngle = currentTurnAngle;
        frontRight.steerAngle = currentTurnAngle;

        UpdateWheel(frontLeft, frontLeftTransform);
        UpdateWheel(frontRight, frontRightTransform);
        UpdateWheel(rearLeft, rearLeftTransform);
        UpdateWheel(rearRight, rearRightTransform);

    }

    private void UpdateWheel(WheelCollider collider, Transform trans)
    {
        collider.GetWorldPose(out Vector3 position, out Quaternion rotation);
        
        trans.SetPositionAndRotation(position, rotation);
    }
}
