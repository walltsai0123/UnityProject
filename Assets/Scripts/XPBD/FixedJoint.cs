using UnityEngine;

namespace XPBD
{
    [RequireComponent(typeof(Body))]
    public class FixedJoint : MonoBehaviour
    {
        public Body thisBody { get; private set; }
        public Body attachedBody;
        private void Awake()
        {
            thisBody = GetComponent<Body>();
        }
        void Start()
        {
            if (thisBody.gameObject.activeInHierarchy && attachedBody.gameObject.activeInHierarchy)
                BackEnd.AddFixedJoint(thisBody.ID, attachedBody.ID);
        }

    }
}

