using UnityEngine;
using Unity.Mathematics;
using XPBD;
public class TestScript : MonoBehaviour
{
    [SerializeField] SoftBody softBody;
    public float accel;
    public float3 dir;
    private void Update()
    {
        if (softBody == null)
            return;

        float3 final = float3.zero;
        if (Input.GetKey(KeyCode.UpArrow))
        {
            final += new float3(0, 0, 1);
        }
        if (Input.GetKey(KeyCode.DownArrow))
        {
            final += new float3(0, 0, -1);
        }
        if (Input.GetKey(KeyCode.LeftArrow))
        {
            final += new float3(-1, 0, 0);
        }
        if (Input.GetKey(KeyCode.RightArrow))
        {
            final += new float3(1, 0, 0);
        }

        dir = final;

        //dir = math.normalizesafe(final, float3.zero);

        //softBody.fext = final * accel;
        softBody.SetForce(final * accel);
    }

}
