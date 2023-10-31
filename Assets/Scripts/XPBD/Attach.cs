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

        private void Awake()
        {
            thisBody = GetComponent<SoftBody>();
        }
        private void Start()
        {
            BackEnd.AttachRigidSoft(attachedBody.ID, thisBody.ID);
        }
    }
    
}

