using System.Collections.Generic;
using UnityEngine;
using System;


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
        public int frame = 0;
        [Space(10)]
        [Header("Simulation Parameters")]
        [SerializeField] bool fixedTimeStep = true;
        public int substeps = 20;
        public bool frameLimit = false;
        public int targetFPS = 60;
        public REAL3 gravity = new(0, -9.81f, 0);
        public Bounds worldBound;

        // Simulation objects and constraints
        public List<Primitive> primitives { get; private set; }
        private List<Body> bodies;
        private List<Rigid> rigidbodies;
        private List<Constraint> constraints;
        private Grabber grabber;

        public SoftBodySystem softBodySystem { get; private set; }
        // Collision
        [Space(10)]
        [Header("Simulation Option")]
        public bool UseTextureFriction = true;
        public Shader textureFrictionShader;
        public bool UseNeoHookeanMaterial = true;
        public bool parallelBlockXPBD = true;
        public bool parallelAttach = true;

        private CollisionDetect collisionDetect;
        private List<CollisionConstraint> collisions;

        //[SerializeField]Terrain terrain;

        public MyTerrain terrain;
        public TerrainSystem terrainSystem;

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
        readonly Timer stepTimer = new();
        readonly Timer bodySolveTimer = new();
        readonly Timer collisionTimer = new();
        readonly Timer constraintTimer = new();
        readonly Timer primitiveTimer = new();  
        public void AddBody(Body b)
        {
            bodies.Add(b);
            if (b.bodyType == Body.BodyType.Rigid)
                rigidbodies.Add((Rigid)b);
            if (b.bodyType == Body.BodyType.Soft)
                softBodySystem.AddSoftBody((SoftBody)b);
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
            rigidbodies = new List<Rigid>();
            constraints = new List<Constraint>();
            collisionDetect = GetComponent<CollisionDetect>();
            collisions = new List<CollisionConstraint>();

            softBodySystem = new SoftBodySystem();

            if (textureFrictionShader == null)
                textureFrictionShader = Shader.Find("Custom/NewUnlitShader");

            grabber = new Grabber(Camera.main);

        }
        private void Start()
        {
            QualitySettings.vSyncCount = 0;
            FirstFrameSetting();
        }

        private void FixedUpdate()
        {
            if(!fixedTimeStep) 
                return;
            //FirstFrameSetting();

            if (Time.frameCount < 10)
                return;
            REAL dt = Time.fixedDeltaTime;

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
            //FirstFrameSetting();
            REAL dt = 1f / targetFPS;

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

            softBodySystem.Init();
        }
        private void SimulationUpdate(REAL dt, int substeps)
        {
            if (pause && !stepOnce)
                return;

            stepTimer.Tic();
            //collisionDetect.CollectCollision(bodies, primitives, dt);
            collisionDetect.CollectCollision(softBodySystem, terrainSystem, dt);
            //collisions = collisionDetect.Collisions;

            if(collisions.Count > 0)
            {
                totalContacts += collisions.Count;
                totalSimLoops++;
            }

            bodySolveTimer.Tic();
            bodySolveTimer.Pause();

            constraintTimer.Tic();
            constraintTimer.Pause();

            primitiveTimer.Tic();
            primitiveTimer.Pause();

            REAL sdt = dt / substeps;

            softBodySystem.SyncBodies();
            for (int step = 0; step < substeps; ++step)
            {

                foreach (Rigid body in rigidbodies)
                    body.PreSolve(sdt, gravity);
                softBodySystem.PreSolve(sdt, gravity);
                
                bodySolveTimer.Resume();

                foreach (Rigid body in rigidbodies)
                    body.Solve(sdt);
                    softBodySystem.Solve(sdt);
                bodySolveTimer.Pause();
                foreach (Constraint C in constraints)
                    C.SolveConstraint(sdt);



                constraintTimer.Resume();
                collisionDetect.SolveCollision(sdt);


                constraintTimer.Pause();

                foreach (Rigid body in rigidbodies)
                    body.PostSolve(sdt);
                    
                softBodySystem.PostSolve(sdt);

                foreach (Constraint C in constraints)
                    C.SolveVelocities(sdt);

                collisionDetect.VelocitySolve(sdt);
            }
            softBodySystem.ClearForce();
            


            int substep2 = 3;
            terrainSystem.FluidToSolid(softBodySystem, dt);
            terrainSystem.SolidToFluid(softBodySystem, dt);

            primitiveTimer.Resume();
            for (int step = 0; step < substep2; ++step)
                terrainSystem.Simulate(dt / substep2, gravity);
            primitiveTimer.Pause();
            terrainSystem.EndFrame();

            foreach (Rigid body in rigidbodies)
                body.EndFrame();
            softBodySystem.EndFrame();

            stepTimer.Toc();
            bodySolveTimer.Toc();
            constraintTimer.Toc();
            primitiveTimer.Toc();
            if(verbose)
            {
                stepTimer.Report("One simulation step", Timer.TimerOutputUnit.TIMER_OUTPUT_MILLISECONDS);
                bodySolveTimer.Report("Body solve time: ", Timer.TimerOutputUnit.TIMER_OUTPUT_MILLISECONDS);
                constraintTimer.Report("Constraint solve time", Timer.TimerOutputUnit.TIMER_OUTPUT_MILLISECONDS);
                primitiveTimer.Report("Primitive solve time", Timer.TimerOutputUnit.TIMER_OUTPUT_MILLISECONDS);
            }

            //Clear step once flag
            stepOnce = false;

            frame++;
        }

        private void OnDestroy()
        {
            if (totalSimLoops > 0 && collisionVerbose)
                Debug.Log("Average contacts: " + totalContacts / totalSimLoops);
            //collisionDetect?.Dispose(); 
            softBodySystem?.Dispose();
        }

        private void OnDrawGizmos()
        {
            if (!GizmosVerbose)
                return;
        }
    }
}

