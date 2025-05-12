using UnityEngine;
using XPBD;
using Unity.Mathematics;

[RequireComponent(typeof(Camera))]
public class CameraFollow : MonoBehaviour
{
    enum Mode
    {
        None,
        Rigid,
        Soft
    };

    [SerializeField] Mode mode = Mode.Soft;
    public SoftBody softBody;
    public Rigid rigid;
    public float dist = 10f;
    // Update is called once per frame
    void Update()
    {
        switch (mode)
        {
            case Mode.Soft:
                FollowSoft();
                break;
            case Mode.Rigid:
                FollowRigid();
                break;
            case Mode.None:
                break;
        }
    }

    private void FollowRigid()
    {
        if(rigid == null) return;
        transform.position = (float3)rigid.Position;

        transform.position += dist * Vector3.back + dist * Vector3.up;

        transform.LookAt((float3)rigid.Position);
    }

    private void FollowSoft()
    {
        if (softBody == null)
            return;

        if (!softBody.gameObject.activeInHierarchy)
            return;

        transform.position = (float3)softBody.X_COM;

        transform.position += dist * Vector3.back + dist * Vector3.up;

        transform.LookAt((float3)softBody.X_COM);
    }
}
