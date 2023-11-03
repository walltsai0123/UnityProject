using UnityEngine;

namespace XPBD
{
    [RequireComponent(typeof(Body))]
    public class FixedJoint : MonoBehaviour
    {
        public Body thisBody { get; private set; }
        public Body attachedBody;

        public int ID1, ID2;

        private void Awake()
        {
            thisBody = GetComponent<Body>();
            Debug.Log("FixedJoint Awake");
        }
        void Start()
        {
            if (thisBody.gameObject.activeInHierarchy && attachedBody.gameObject.activeInHierarchy)
            {
                ID1 = thisBody.ID;
                ID2 = attachedBody.ID;
                
                BackEnd.AddFixedJoint(ID1, ID2);
            }
                
        }

    }
}

