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
        public Vector3 gravity = new(0, -9.81f, 0);
        private List<Body> bodies;
        private List<Constraint> constraints;
        private Thread _workerThread;

        public void AddBody(Body b)
        {
            bodies.Add(b);
        }
        public void AddConstraints(Constraint c)
        {
            constraints.Add(c);
        }
        private void Awake()
        {
            if (get)
            {
                Debug.LogWarning("Simulation instance already exists.");
                enabled = false;
                return;
            }
            get = this;
            bodies = new List<Body>();
            constraints = new List<Constraint>();
            //BackEnd.XPBDSimInit();
            Debug.Log("Simulation Awake");
        }

        private void FixedUpdate()
        {
            //if (_workerThread != null && !_workerThread.IsAlive)
            //    PostExecuteThread();
            //if (_workerThread == null)
            //    ExecuteThread(Time.fixedDeltaTime);

            //BackEnd.XPBDSimUpdate(Time.fixedDeltaTime, substeps);
            //Debug.Log("Simulation Update");
            float dt = Time.fixedDeltaTime;
            SimulationUpdate(dt, substeps);
        }

        private void SimulationUpdate(float dt, int substeps)
        {
            float sdt = dt / substeps;
            foreach (Body body in bodies)
                body.CollectCollision(dt);

            //Debug.Log("Loop substep start");
            for (int step = 0; step < substeps; ++step)
            {
                foreach (Body body in bodies)
                    body.PreSolve(sdt, gravity);

                foreach (Body body in bodies)
                    body.Solve(sdt);

                foreach (Constraint C in constraints)
                    C.SolveConstraint(sdt);

                foreach (Body body in bodies)
                    body.PostSolve(sdt);

                foreach (Body body in bodies)
                    body.VelocitySolve(sdt);
            }
            foreach (Body body in bodies)
                body.EndFrame();
            //Debug.Log("Loop substep end");
        }
        private void ExecuteThread(float dt)
        {
            Assert.IsTrue(_workerThread == null);

            //_workerThread = new Thread(() => { BackEnd.XPBDSimUpdate(dt, substeps); });
            _workerThread = new Thread(() => { SimulationUpdate(dt, substeps); });
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
            BackEnd.XPBDSimDelete();
            Debug.Log("Simulation Destroy");
        }
    }
}

