using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace XPBD
{
    public class PosConstraint : MonoBehaviour
    {
        public Body body1, body2;
        public Vector3 r1, r2;
        public float length = 0f;
        public float comp = 0f;

        private void Start()
        {
            if(body1.gameObject.activeInHierarchy && body2.gameObject.activeInHierarchy)
            {
                BackEnd.AddPosConstraints(body1.ID, body2.ID, r1, r2, length, comp);
                Debug.Log("PosConstraint start");
            }
                
        }
    }

}

