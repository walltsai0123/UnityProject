using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

namespace XPBD
{
    public class MyWheelController : MonoBehaviour
    {
        [SerializeField] Rigid frontLeft;
        [SerializeField] Rigid frontRight;
        [SerializeField] Rigid rearLeft;
        [SerializeField] Rigid rearRight;

        private Rigid carBody;
        public float accelration = 500f;
        public float brakingForce = 300f;
        public float maxTurnAngle = 15f;

        private float currentAccel = 0f;
        private float currentBrakeForce = 0f;
        private float currentTurnAngle = 0f;

        private Vector3 axis = Vector3.right;

        private void Awake()
        {
            carBody = GetComponent<Rigid>();
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

            float3 worldAxis = math.rotate(carBody.Rotation, axis);
            float3 totalForce = (currentAccel - currentBrakeForce) * worldAxis;

            frontLeft.Tau = totalForce;
            frontRight.Tau = totalForce;
            rearLeft.Tau = totalForce;
            rearRight.Tau = totalForce;
        }
    }

}
