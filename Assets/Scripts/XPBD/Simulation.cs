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
        public int substeps = 20;
        public Vector3 gravity = new(0, -9.81f, 0);

        // Simulation objects and constraints
        public List<Primitive> primitives { get; private set; }
        private List<Body> bodies;
        private List<Constraint> constraints;

        // Collision
        public bool UseTextureFriction = true;
        public Shader textureFrictionShader;
        private CollisionDetect collisionDetect;
        private List<CollisionConstraint> collisions;

        private Grabber grabber;
        public bool pause = false;
        public bool stepOnce = false;

        private int totalContacts = 0;
        private int totalSimLoops = 0;

        public bool UseNeoHookeanMaterial = true;

        public int targetFPS = 60;

        // Timer
        Timer stepTimer = new();
        Timer bodySolveTimer = new();

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

            if (textureFrictionShader == null)
                textureFrictionShader = Shader.Find("Custom/NewUnlitShader");

            grabber = new Grabber(Camera.main);
        }
        private void Start()
        {
            QualitySettings.vSyncCount = 0;
            Application.targetFrameRate = targetFPS;
        }
        private void Update()
        {
            if(Time.frameCount < 10)
                return;
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
            float dt = 1f / targetFPS;
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


            collisionDetect.CollectCollision(bodies, primitives, dt);
            collisions = collisionDetect.Collisions;

            if(collisions.Count > 0)
            {
                totalContacts += collisions.Count;
                totalSimLoops++;
            }

            bodySolveTimer.Tic();
            bodySolveTimer.Pause();
            float sdt = dt / substeps;
            for (int step = 0; step < substeps; ++step)
            {
                foreach(Primitive p in primitives) 
                {
                    p.Simulate(sdt);
                }

                foreach (Body body in bodies)
                    if(!body.isFixed)
                        body.PreSolve(sdt, gravity);
                
                bodySolveTimer.Resume();

                foreach (Body body in bodies)
                    if (!body.isFixed)
                        body.Solve(sdt);

                bodySolveTimer.Pause();

                foreach (Constraint C in constraints)
                    C.ResetLambda();

                foreach (Constraint C in constraints)
                    C.SolveConstraint(sdt);

                collisionDetect.SetFrictionCoef();
                foreach (CollisionConstraint collision in collisions)
                    collision.SolveCollision(sdt);
                
                foreach (Body body in bodies)
                    if (!body.isFixed)
                        body.PostSolve(sdt);

                foreach (Constraint C in constraints)
                    C.SolveVelocities(sdt);

                foreach (CollisionConstraint collision in collisions)
                    collision.VelocitySolve(sdt);

                foreach (Primitive p in primitives)
                    p.ApplyVelocity();
            }
            foreach (Primitive p in primitives)
                p.UpdateVisual();
            
            foreach (Body body in bodies)
                body.EndFrame();

            stepTimer.Toc();
            stepTimer.Report("One simulation step", Timer.TimerOutputUnit.TIMER_OUTPUT_MILLISECONDS);

            bodySolveTimer.Toc();
            bodySolveTimer.Report("Body solve time: ", Timer.TimerOutputUnit.TIMER_OUTPUT_MILLISECONDS);
        }

        private void OnDestroy()
        {
            if (totalSimLoops > 0)
                Debug.Log("Average contacts: " + totalContacts / totalSimLoops);
            collisionDetect.Dispose(); 
        }
    }
}

