using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace XPBD
{
    [RequireComponent(typeof(SoftBody))]
    public class Attach : MonoBehaviour
    {
        public SoftBody thisBody { get; private set; }
        public Rigid attachedBody;

        public int ID1, ID2;

        private void Awake()
        {
            thisBody = GetComponent<SoftBody>();
            Debug.Log("Attach Awake");
        }
        private void Start()
        {
            
            if (thisBody.gameObject.activeInHierarchy && attachedBody.gameObject.activeInHierarchy)
            {
                ID1 = attachedBody.ID;
                ID2 = thisBody.ID;
                BackEnd.AttachRigidSoft(ID1, ID2);
            }
                
        }
    }
    
}

