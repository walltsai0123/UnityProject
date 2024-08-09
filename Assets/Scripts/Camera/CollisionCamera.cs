using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Assertions;
using UnityEngine.Rendering;

[RequireComponent(typeof(Camera))]
public class CollisionCamera : MonoBehaviour
{
    public Camera Cam { get; private set; }

    public int layer;
    [SerializeField] public Shader shader;
    public RenderTexture renderTexture;
    CommandBuffer commandBuffer;
    void Start()
    {
        Cam = GetComponent<Camera>();
        //Cam.cullingMask = 1 << layer;

        Assert.IsTrue(shader != null);
        Assert.IsTrue(renderTexture != null);

        renderTexture.Release();

        commandBuffer = new CommandBuffer();
        commandBuffer.name = gameObject.name;

        Cam.SetReplacementShader(shader, "");
    }

    public void RenderToTexture()
    {
        Cam.targetTexture = renderTexture;
        Cam.RenderWithShader(shader, "");
        Cam.targetTexture = null;
    }

    public void DrawToTexture(Mesh mesh, Material mat, Matrix4x4 matrix)
    {
        Rect viewport = Cam.rect;
        viewport.x *= renderTexture.width;
        viewport.y *= renderTexture.height;
        viewport.width *= renderTexture.width;
        viewport.height *= renderTexture.height;
        
        commandBuffer.Clear();
        commandBuffer.name = gameObject.name;
        commandBuffer.SetRenderTarget(renderTexture);
        commandBuffer.SetViewport(viewport);
        commandBuffer.SetViewProjectionMatrices(Cam.worldToCameraMatrix, Cam.projectionMatrix);
        commandBuffer.DrawMesh(mesh, matrix, mat);
        Graphics.ExecuteCommandBuffer(commandBuffer);
    }

    public void SetViewPort(float x, float y, float w, float h)
    {
        //Cam.pixelRect = new Rect(x, y, w, h);
        Cam.rect = new Rect(x,y,w,h);
    }
}
