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
    public Shader shader { get; set; }
    public RenderTexture renderTexture;
    CommandBuffer commandBuffer;
    void Start()
    {
        Cam = GetComponent<Camera>();
        Cam.enabled = false;
        //Cam.cullingMask = 1 << layer;

       // Assert.IsTrue(shader != null);
        Assert.IsTrue(renderTexture != null);

        renderTexture.Release();

        commandBuffer = new CommandBuffer();
        commandBuffer.name = gameObject.name;

        //Cam.SetReplacementShader(shader, "");
    }

    public void RenderToTexture()
    {
        Cam.targetTexture = renderTexture;
        Cam.RenderWithShader(shader, "");
        //Cam.targetTexture = null;
    }

    public void DrawToTexture(Mesh mesh, Material mat, Matrix4x4 matrix, bool instancing = false)
    {
        Rect viewport = Cam.rect;
        viewport.x *= renderTexture.width;
        viewport.y *= renderTexture.height;
        viewport.width *= renderTexture.width;
        viewport.height *= renderTexture.height;

        //viewport = new Rect(0,0, renderTexture.width, renderTexture.height);
        
        commandBuffer.Clear();
        commandBuffer.name = gameObject.name;
        commandBuffer.SetRenderTarget(renderTexture);
        commandBuffer.SetViewport(viewport);
        commandBuffer.SetViewProjectionMatrices(Cam.worldToCameraMatrix, Cam.projectionMatrix);

        if(instancing)
        {
            Matrix4x4[] matrix4X4s = { matrix };
            commandBuffer.DrawMeshInstanced(mesh, 0, mat, -1, matrix4X4s);
        }
        else
        {
            commandBuffer.DrawMesh(mesh, matrix, mat);
        }
        Graphics.ExecuteCommandBuffer(commandBuffer);
    }

    public void SetViewPort(float x, float y, float w, float h)
    {
        //Cam.pixelRect = new Rect(x, y, w, h);
        Cam.rect = new Rect(x,y,w,h);
    }
}
