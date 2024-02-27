using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Assertions;

[RequireComponent(typeof(Camera))]
public class CollisionCamera : MonoBehaviour
{
    public Camera Cam { get; private set; }

    public int layer;
    [SerializeField] Shader shader;
    public RenderTexture renderTexture;

    void Start()
    {
        Cam = GetComponent<Camera>();
        Cam.cullingMask = 1 << layer;
        //if(shader != null)
        //    Cam.SetReplacementShader(shader, "");
        Assert.IsTrue(shader != null);
        Assert.IsTrue(renderTexture != null);

        renderTexture.Release();
    }

    public void RenderToTexture()
    {
        //Cam.targetTexture = renderTexture;
        //Cam.Render();
        //Cam.targetTexture = null;
        Cam.RenderWithShader(shader, "");
    }

    public void SetViewPort(float x, float y, float w, float h)
    {
        Cam.pixelRect = new Rect(x, y, w, h);
    }
}
