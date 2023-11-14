using System.Collections;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;
using UnityEngine.Assertions;

namespace XPBD
{
    public class Simulation : MonoBehaviour
    {
        public static Simulation get;
        public int substeps = 10;
        private Thread _workerThread;
        private void Awake()
        {
            if (get)
            {
                Debug.LogWarning("Simulation instance already exists.");
                enabled = false;
                return;
            }
            get = this;
            BackEnd.XPBDSimInit();
            Debug.Log("Simulation Awake");
        }

        private void FixedUpdate()
        {
            if (_workerThread != null && !_workerThread.IsAlive)
                PostExecuteThread();
            if (_workerThread == null)
                ExecuteThread(Time.fixedDeltaTime);

            //BackEnd.XPBDSimUpdate(Time.fixedDeltaTime, substeps);
            //Debug.Log("Simulation Update");
        }
        private void ExecuteThread(float dt)
        {
            Assert.IsTrue(_workerThread == null);

            _workerThread = new Thread(() => { BackEnd.XPBDSimUpdate(dt, substeps); });
            _workerThread.Name = "SimWorker";
            _workerThread.Start();
        }

        private void PostExecuteThread()
        {
            Assert.IsTrue(!_workerThread.IsAlive);

            _workerThread.Join();
            _workerThread = null;
        }

        private void OnDestroy()
        {
            if (_workerThread != null)
            {
                _workerThread.Join();
                _workerThread = null;
            }
            //BackEnd.DeleteSoftBody();
            BackEnd.XPBDSimDelete();
            Debug.Log("Simulation Destroy");
        }
    }
}

