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
    private int display;

    void Start()
    {
        Cam = GetComponent<Camera>();
        Cam.cullingMask = 1 << layer;
        if(shader != null)
            Cam.SetReplacementShader(shader, "");
        display = Cam.targetDisplay;

        Assert.IsTrue(renderTexture != null);
    }

    public void RenderToTexture()
    {
        Cam.targetTexture = renderTexture;
    }

    public void RenderToDisplay()
    {
        Cam.targetTexture = null;
    }

    public void Render()
    {
        Cam.Render();
    }
}
