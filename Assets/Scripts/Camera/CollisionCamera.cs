using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Camera))]
public class CollisionCamera : MonoBehaviour
{
    Camera thisCamera;
    [SerializeField] Shader shader;
    //Camera camera;
    // Start is called before the first frame update
    void Start()
    {
        thisCamera = GetComponent<Camera>();
        if(shader != null)
            thisCamera.SetReplacementShader(shader, "");
    }

}
