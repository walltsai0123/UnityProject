using System;
using UnityEngine;
[CreateAssetMenu(fileName = "New ground material", menuName = "Create Ground Material", order = 1)]
public class GroundMaterial : ScriptableObject
{
    public float YoungModulus;
    [Range(0, 0.5f)] public float PoissonRatio;

    public float n;
    public float k_c;
    public float k_phi;
    public float cohesion;
    public float frictionAngle;
    public float K;

    public float Nc
    {
        get
        {
            int floor = Mathf.FloorToInt(frictionAngle);
            int ceiling = Mathf.CeilToInt(frictionAngle);
            float nc1 = N_c[floor];
            float nc2 = N_c[ceiling];
            return Mathf.Lerp(frictionAngle - floor, nc1, nc2);
        }
    }
    public float Nq
    {
        get
        {
            int floor = Mathf.FloorToInt(frictionAngle);
            int ceiling = Mathf.CeilToInt(frictionAngle);
            float nq1 = N_q[floor];
            float nq2 = N_q[ceiling];
            return Mathf.Lerp(frictionAngle - floor, nq1, nq2);
        }
    }
    public float Nr
    {
        get
        {
            int floor = Mathf.FloorToInt(frictionAngle);
            int ceiling = Mathf.CeilToInt(frictionAngle);
            float ngamma1 = N_gamma[floor];
            float ngamma2 = N_gamma[ceiling];
            return Mathf.Lerp(frictionAngle - floor, ngamma1, ngamma2);
        }
    }
    public float unitWeight;
    public float frictionAngleInRadian => Mathf.Deg2Rad * frictionAngle;

    static readonly float[] N_c = new float[]
    {
        5.70f, 6.00f, 6.30f, 6.62f, 6.97f, 7.34f, 7.73f, 8.15f, 8.60f, 9.09f,
        9.61f, 10.16f, 10.76f, 11.41f, 12.11f, 12.86f, 13.68f, 14.60f, 15.12f, 16.56f,
        17.69f, 18.92f, 20.27f, 21.75f, 23.36f, 25.13f, 27.09f, 29.24f, 31.61f, 34.24f,
        37.16f, 40.41f, 44.04f, 48.09f, 52.64f, 57.75f, 63.53f, 70.01f, 77.50f, 85.97f,
        95.66f, 106.81f, 119.67f, 134.58f, 151.95f, 172.28f, 196.22f, 224.55f, 258.28f, 298.71f,
        347.50f
    };

    static readonly float[] N_q = new float[]
    {
        1.00f, 1.10f, 1.22f, 1.35f, 1.49f, 1.64f, 1.81f, 2.00f, 2.21f, 2.44f,
        2.69f, 2.98f, 3.29f, 3.63f, 4.02f, 4.45f, 4.92f, 5.45f, 6.04f, 6.70f,
        7.44f, 8.26f, 9.19f, 10.23f, 11.40f, 12.72f, 14.21f, 15.90f, 17.81f, 19.98f,
        22.46f, 25.28f, 28.52f, 32.23f, 36.50f, 41.44f, 47.16f, 53.80f, 61.55f, 70.61f,
        81.27f, 93.85f, 108.75f, 126.50f, 147.74f, 173.28f, 204.19f, 241.80f, 287.85f, 344.63f,
        415.14f
    };

    static readonly float[] N_gamma = new float[]
    {
        0.00f, 0.01f, 0.04f, 0.06f, 0.10f, 0.14f, 0.20f, 0.27f, 0.35f, 0.44f,
        0.56f, 0.69f, 0.85f, 1.04f, 1.26f, 1.52f, 1.82f, 2.18f, 2.59f, 3.07f,
        3.64f, 4.31f, 5.09f, 6.00f, 7.08f, 8.34f, 9.84f, 11.60f, 13.70f, 16.18f,
        19.13f, 22.65f, 26.87f, 31.94f, 38.04f, 45.41f, 54.36f, 65.27f, 78.61f, 95.03f,
        115.31f, 140.51f, 171.99f, 211.56f, 261.60f, 325.34f, 407.11f, 512.84f, 650.67f, 831.99f,
        1072.80f
    };
}
