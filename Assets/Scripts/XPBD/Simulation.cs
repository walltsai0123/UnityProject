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
        private List<CollisionConstraint> collisions;

        private Grabber grabber;


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
                enabled = false;
                return;
            }
            get = this;
            primitives = new List<Primitive>();
            bodies = new List<Body>();
            constraints = new List<Constraint>();
            collisions = new List<CollisionConstraint>();

            grabber = new Grabber(Camera.main);

            // BackEnd.XPBDSimInit();
            // Debug.Log("Simulation Awake");
        }

        private void FixedUpdate()
        {
            float dt = Time.fixedDeltaTime;
            SimulationUpdate(dt, substeps);
        }

        private void Update()
        {
            grabber.MoveGrab();
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
            float sdt = dt / substeps;
            //foreach (Body body in bodies)
            //    foreach (Primitive primitive in primitives)
            //        body.CollectCollision(dt, primitive);
            collisions.Clear();
            foreach (Body body in bodies)
            {
                if (!body.EnableContact)
                    continue;

                if(body.bodyType == Body.BodyType.Rigid)
                {
                    Rigid rigid = (Rigid) body;
                    collisions.AddRange(CollisionDetect.RigidCollision(rigid));
                }
                else
                {
                    foreach (Primitive primitive in primitives)
                    {
                        collisions.AddRange(CollisionDetect.CollectCollision(body, primitive, dt));
                    }
                }
            }

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
        }

        private void OnDestroy()
        {
            // BackEnd.XPBDSimDelete();
            Debug.Log("Simulation Destroy");
        }
    }
}

