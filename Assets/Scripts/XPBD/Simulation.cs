using System.Collections.Generic;
using UnityEngine;

#if USE_FLOAT
using REAL = System.Single;
using REAL2 = Unity.Mathematics.float2;
using REAL3 = Unity.Mathematics.float3;
using REAL4 = Unity.Mathematics.float4;
using REAL2x2 = Unity.Mathematics.float2x2;
using REAL3x3 = Unity.Mathematics.float3x3;
using REAL3x4 = Unity.Mathematics.float3x4;
#else
using REAL = System.Double;
using REAL2 = Unity.Mathematics.double2;
using REAL3 = Unity.Mathematics.double3;
using REAL4 = Unity.Mathematics.double4;
using REAL2x2 = Unity.Mathematics.double2x2;
using REAL3x3 = Unity.Mathematics.double3x3;
using REAL3x4 = Unity.Mathematics.double3x4;
#endif

namespace XPBD
{
    public class Simulation : MonoBehaviour
    {
        public static Simulation get;

        [Space(10)]
        [Header("Simulation Parameters")]
        [SerializeField] bool fixedTimeStep = true;
        public int substeps = 20;
        public bool frameLimit = false;
        public int targetFPS = 60;
        public REAL3 gravity = new(0, -9.81f, 0);

        // Simulation objects and constraints
        public List<Primitive> primitives { get; private set; }
        private List<Body> bodies;
        private List<SoftBody> softBodies;
        private List<Constraint> constraints;
        private Grabber grabber;

        public SoftBodySystem softbodySystem;

        // Collision
        [Space(10)]
        [Header("Simulation Option")]
        public bool UseTextureFriction = true;
        public Shader textureFrictionShader;
        public bool UseNeoHookeanMaterial = true;
        public bool GPUSolver = true;

        private CollisionDetect collisionDetect;
        private CollisionDetectGPU collisionDetectGPU;
        private List<CollisionConstraint> collisions;

        [SerializeField]Terrain terrain;

        [Space(10)]
        [Header("State Control")]
        public bool pause = false;
        public bool stepOnce = false;

        [Space(10)]
        [Header("Verbose")]
        public bool verbose = false;
        public bool collisionVerbose = false;
        public bool GizmosVerbose = false;

        private int totalContacts = 0;
        private int totalSimLoops = 0;
        private bool firstFrame = true;
        // Timer
        Timer stepTimer = new();
        Timer bodySolveTimer = new();

        public void AddBody(Body b)
        {
            bodies.Add(b);
            if (b.bodyType == Body.BodyType.Soft)
                softBodies.Add((SoftBody)b);
        }
        public void RemoveBody(Body b)
        {
            bodies.Remove(b);
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
            softBodies = new List<SoftBody>();
            constraints = new List<Constraint>();
            collisionDetect = new CollisionDetect();
            collisions = new List<CollisionConstraint>();

            collisionDetectGPU = GetComponent<CollisionDetectGPU>();
            if(collisionDetectGPU == null)
            {
                collisionDetectGPU = gameObject.AddComponent<CollisionDetectGPU>();
            }

            if (textureFrictionShader == null)
                textureFrictionShader = Shader.Find("Custom/NewUnlitShader");

            // Create softbody system if not exist
            if(softbodySystem == null)
            {
                GameObject gSB = new GameObject("GlobalSoftBody");
                gSB.transform.parent = transform;
                gSB.AddComponent<SoftBodySystem>();
                softbodySystem = gSB.GetComponent<SoftBodySystem>();
            }

            grabber = new Grabber(Camera.main);

        }
        private void Start()
        {
            QualitySettings.vSyncCount = 0;
        }

        private void FixedUpdate()
        {
            if(!fixedTimeStep) 
                return;
            FirstFrameSetting();

            if (Time.frameCount < 10)
                return;
            REAL dt = Time.fixedDeltaTime;

            if(GPUSolver)
                SimulationUpdateGPU(dt, substeps);
            else
                SimulationUpdate(dt, substeps);
        }
        private void Update()
        {
            Application.targetFrameRate = (frameLimit) ? targetFPS : -1;

            grabber.MoveGrab();
            if (Input.GetKeyDown(KeyCode.P))
            {
                pause = !pause;
            }
            if (Input.GetKeyDown(KeyCode.N))
            {
                stepOnce = true;
            }

            if (fixedTimeStep || Time.frameCount < 10)
                return;
            FirstFrameSetting();
            REAL dt = 1f / targetFPS;
            if (GPUSolver)
                SimulationUpdateGPU(dt, substeps);
            else
                SimulationUpdate(dt, substeps);
        }

        private void LateUpdate()
        {
            //if (Input.GetMouseButtonDown(0))
            //{
            //    List<IGrabbable> temp = new List<IGrabbable>(bodies);

            //    grabber.StartGrab(temp);
            //}

            //if (Input.GetMouseButtonUp(0))
            //{
            //    grabber.EndGrab();
            //}
        }

        private void FirstFrameSetting()
        {
            if (!firstFrame)
                return;

            firstFrame = false;
            if(GPUSolver)
                softbodySystem?.CollectSoftBodies(softBodies);
        }
        private void SimulationUpdate(REAL dt, int substeps)
        {
            if (pause && !stepOnce)
                return;

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

            REAL sdt = dt / substeps;
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

                collisionDetect?.SetFrictionCoef();
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
                    p.ApplyVelocity(sdt);
            }
            foreach (Primitive p in primitives)
                p.UpdateVisual();
            
            foreach (Body body in bodies)
                body.EndFrame();

            stepTimer.Toc();
            bodySolveTimer.Toc();

            if(verbose)
            {
                stepTimer.Report("One simulation step", Timer.TimerOutputUnit.TIMER_OUTPUT_MILLISECONDS);
                bodySolveTimer.Report("Body solve time: ", Timer.TimerOutputUnit.TIMER_OUTPUT_MILLISECONDS);
            }

            //Clear step once flag
            stepOnce = false;
        }

        private void SimulationUpdateGPU(REAL dt, int substeps)
        {
            if (pause && !stepOnce)
                return;

            Timer stepTimer = new(); stepTimer.Tic();

            collisionDetectGPU.CollectCollisions(softbodySystem, terrain, dt);
            REAL sdt = dt / substeps;
            for (int step = 0; step < substeps; ++step)
            {
                //foreach (Primitive p in primitives)
                //{
                //    p.Simulate(sdt);
                //}

                softbodySystem.PreSolve(sdt, gravity);
                softbodySystem.Solve(sdt);

                collisionDetectGPU.CollisionSolve(sdt);

                softbodySystem.PostSolve(sdt);

                collisionDetectGPU.CollisionVelocitySolve(sdt);

                //foreach (Primitive p in primitives)
                //    p.ApplyVelocity(sdt);
            }

            softbodySystem.EndFrame();


            stepTimer.Toc();

            if (verbose)
            {
                stepTimer.Report("One simulation step", Timer.TimerOutputUnit.TIMER_OUTPUT_MILLISECONDS);
            }


            //Clear step once flag
            stepOnce = false;
        }
        private void OnDestroy()
        {
            if (totalSimLoops > 0 && collisionVerbose)
                Debug.Log("Average contacts: " + totalContacts / totalSimLoops);
            collisionDetect?.Dispose(); 
        }

        private void OnDrawGizmos()
        {
            if (!GizmosVerbose)
                return;
        }

    }
}

