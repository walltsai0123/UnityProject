using System;
using UnityEngine;
[CreateAssetMenu(fileName = "New ground material", menuName = "Create Ground Material", order = 1)]
public class GroundMaterial : ScriptableObject
{
    public float YoungModulus;
    public float k_c;
    public float k_phi;
    public float n;
    [Range(0, 0.5f)]public float PoissonRatio;
    public float cohesion;
    public float frictionAngle;
    public float K;

    public float Nc;
    public float Nq;
    public float Nr;
    public float unitWeight;
    public float frictionAngleInRadian => Mathf.Deg2Rad * frictionAngle;
}
