using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using static UnityEngine.ParticleSystem;
using XPBD;

public class Tire : MonoBehaviour
{
    public GameObject particlePrefab;
    public float radius = 0.5f;
    public int segments = 12;
    public int ringCount = 3;
    public float ringSpacing = 0.2f;

    private List<Particle> particles = new();
    private List<TetraVolumeConstraint> constraints = new();

    void Start()
    {
        CreateMultiRingTetraWheel();
    }


    void CreateMultiRingTetraWheel()
    {
        particlePrefab = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        particlePrefab.transform.localScale = Vector3.one * 0.1f;
        // Generate ringCount rings along Z-axis
        for (int ring = 0; ring < ringCount; ring++)
        {
            float z = ring * ringSpacing;
            for (int i = 0; i < segments; i++)
            {
                float angle = 2 * Mathf.PI * i / segments;
                Vector3 pos = new Vector3(Mathf.Cos(angle), Mathf.Sin(angle), 0) * radius + new Vector3(0, 0, z);
                particles.Add(new Particle(pos, 1.0f, null));
            }
        }

        // Add top and bottom caps
        Vector3 top = new Vector3(0, 0, ringSpacing * ringCount);
        Vector3 bottom = new Vector3(0, 0, -ringSpacing);
        int topIndex = particles.Count;
        int bottomIndex = particles.Count + 1;
        particles.Add(new Particle(top, 1.0f, null));
        particles.Add(new Particle(bottom, 1.0f, null));

        // Connect rings with tetrahedra
        for (int ring = 0; ring < ringCount - 1; ring++)
        {
            int ring0 = ring * segments;
            int ring1 = (ring + 1) * segments;
            for (int i = 0; i < segments; i++)
            {
                int a = ring0 + i;
                int b = ring0 + (i + 1) % segments;
                int c = ring1 + i;
                int d = ring1 + (i + 1) % segments;
                constraints.Add(new TetraVolumeConstraint(a, b, c, d, particles));
                constraints.Add(new TetraVolumeConstraint(a, c, d, b, particles));
            }
        }

        // Cap connections
        int topRingStart = (ringCount - 1) * segments;
        for (int i = 0; i < segments; i++)
        {
            int a = topRingStart + i;
            int b = topRingStart + (i + 1) % segments;
            constraints.Add(new TetraVolumeConstraint(topIndex, a, b, b, particles));
        }

        for (int i = 0; i < segments; i++)
        {
            int a = i;
            int b = (i + 1) % segments;
            constraints.Add(new TetraVolumeConstraint(bottomIndex, a, b, b, particles));
        }
    }
    private void OnDrawGizmos()
    {
        Gizmos.color = Color.green;

        foreach(var c in constraints)
        {
            Gizmos.DrawLine(particles[c.i1].position, particles[c.i2].position);
            Gizmos.DrawLine(particles[c.i1].position, particles[c.i3].position);
            Gizmos.DrawLine(particles[c.i1].position, particles[c.i4].position);
            Gizmos.DrawLine(particles[c.i2].position, particles[c.i3].position);
            Gizmos.DrawLine(particles[c.i2].position, particles[c.i4].position);
            Gizmos.DrawLine(particles[c.i3].position, particles[c.i4].position);
        }
    }
    class Particle
    {
        public Vector3 position, prevPosition, velocity;
        public float inverseMass;
        public GameObject visual;

        public Particle(Vector3 pos, float invMass, GameObject vis)
        {
            position = prevPosition = pos;
            inverseMass = invMass;
            velocity = Vector3.zero;
            visual = vis;
        }
    }
    class TetraVolumeConstraint
    {
        public int i1, i2, i3, i4;
        float restVolume;

        public TetraVolumeConstraint(int a, int b, int c, int d, List<Particle> particles)
        {
            i1 = a; i2 = b; i3 = c; i4 = d;
            restVolume = ComputeVolume(particles);
        }

        float ComputeVolume(List<Particle> p)
        {
            Vector3 a = p[i1].position;
            Vector3 b = p[i2].position;
            Vector3 c = p[i3].position;
            Vector3 d = p[i4].position;
            return Mathf.Abs(Vector3.Dot(a - d, Vector3.Cross(b - d, c - d))) / 6f;
        }

        public void Solve(List<Particle> p)
        {
            // XPBD volume constraint simplified: here we just pull points based on volume difference
            float currentVol = ComputeVolume(p);
            float diff = (currentVol - restVolume) / restVolume;
            float stiffness = 0.5f;

            Vector3 grad1 = Vector3.Cross(p[i2].position - p[i4].position, p[i3].position - p[i4].position);
            Vector3 grad2 = Vector3.Cross(p[i3].position - p[i4].position, p[i1].position - p[i4].position);
            Vector3 grad3 = Vector3.Cross(p[i1].position - p[i4].position, p[i2].position - p[i4].position);
            Vector3 grad4 = -grad1 - grad2 - grad3;

            Vector3[] grads = { grad1, grad2, grad3, grad4 };
            int[] ids = { i1, i2, i3, i4 };

            float denom = 0f;
            for (int i = 0; i < 4; i++)
                denom += p[ids[i]].inverseMass * grads[i].sqrMagnitude;

            if (denom < 1e-6f) return;

            float lambda = stiffness * diff / denom;
            for (int i = 0; i < 4; i++)
                p[ids[i]].position -= lambda * p[ids[i]].inverseMass * grads[i];
        }
    }
}
