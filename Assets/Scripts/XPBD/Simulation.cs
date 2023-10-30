using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace XPBD
{
    public class Simulation : MonoBehaviour
    {
        public static Simulation get;
        public int substeps = 10;
        private void Awake()
        {
            if (get)
            {
                Debug.LogWarning("Simulation instance already exists.");
                enabled = false;
                return;
            }
            get = this;
            Debug.Log("Simulation Awake");
        }

        private void FixedUpdate()
        {
            BackEnd.XPBDSimUpdate(Time.fixedDeltaTime, substeps);
        }

        private void OnDestroy()
        {
            //BackEnd.DeleteSoftBody();
            BackEnd.XPBDSimDelete();
            Debug.Log("Simulation Destroy");
        }
    }
}

