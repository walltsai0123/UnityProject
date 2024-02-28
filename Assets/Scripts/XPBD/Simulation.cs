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
        private List<Primitive> primitives;
        private List<Body> bodies;
        private List<Constraint> constraints;

        // Collision
        private CollisionDetect collisionDetect;
        private List<CollisionConstraint> collisions;

        private Grabber grabber;
        private bool pause = false;
        private bool stepOnce = false;

        private int totalContacts = 0;
        private int totalSimLoops = 0;

        public bool UseNeoHookeanMaterial = true;

        // Timer
        //Timer stepTimer;

        public void AddPrimitive(Primitive p)
        {
            primitives.Add(p);
        }
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
                Destroy(gameObject);
                return;
            }
            get = this;
            primitives = new List<Primitive>();
            bodies = new List<Body>();
            constraints = new List<Constraint>();
            collisionDetect = new CollisionDetect();
            collisions = new List<CollisionConstraint>();

            grabber = new Grabber(Camera.main);

            // BackEnd.XPBDSimInit();
            Debug.Log("Simulation Awake");
        }

        private void Update()
        {
            grabber.MoveGrab();

            if (Input.GetKeyDown(KeyCode.P))
            {
                pause = !pause;
            }
            if (Input.GetKeyDown(KeyCode.N))
            {
                stepOnce = true;
            }

            if (pause && !stepOnce)
                return;
            float dt = Time.deltaTime;
            SimulationUpdate(dt, substeps);

            //Clear step once flag
            stepOnce = false;
        }

        private void LateUpdate()
        {
            if (Input.GetMouseButtonDown(0))
            {
                List<IGrabbable> temp = new List<IGrabbable>(bodies);

                grabber.StartGrab(temp);
            }

            if (Input.GetMouseButtonUp(0))
            {
                grabber.EndGrab();
            }
        }

        private void SimulationUpdate(float dt, int substeps)
        {
            Timer stepTimer = new(); stepTimer.Tic();
            //collisions.Clear();
            collisionDetect.CollectCollision(bodies, primitives, dt);
            collisions = collisionDetect.Collisions;

            if(collisions.Count > 0)
            {
                totalContacts += collisions.Count;
                totalSimLoops++;
            }
            

            float sdt = dt / substeps;
            for (int step = 0; step < substeps; ++step)
            {
                foreach (Body body in bodies)
                    body.PreSolve(sdt, gravity);

                foreach (Body body in bodies)
                    body.Solve(sdt);

                foreach (Constraint C in constraints)
                    C.SolveConstraint(sdt);

                foreach (CollisionConstraint collision in collisions)
                    collision.SolveCollision(sdt);

                foreach (Body body in bodies)
                    body.PostSolve(sdt);

                foreach (Body body in bodies)
                    body.VelocitySolve(sdt);

                foreach (CollisionConstraint collision in collisions)
                    collision.VelocitySolve(sdt);
            }
            foreach (Body body in bodies)
                body.EndFrame();

            stepTimer.Toc();
            // stepTimer.Report("One simulation step", Timer.TimerOutputUnit.TIMER_OUTPUT_MILLISECONDS);
        }

        private void OnDestroy()
        {
            // BackEnd.XPBDSimDelete();
            if (totalSimLoops > 0)
                Debug.Log("Average contacts: " + totalContacts / totalSimLoops);
            Debug.Log("Simulation Destroy");
        }
    }
}

