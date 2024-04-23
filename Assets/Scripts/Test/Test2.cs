using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering;

public class Test2 : MonoBehaviour
{
    public Mesh mesh;
    public Material material;
    public Shader shader;
    public Transform other;
    public RenderTexture renderTexture;
    private CommandBuffer m_CommandBuffer;
    readonly Matrix4x4 m_LookMatrix = Matrix4x4.TRS(new Vector3(0, 0, -1), Quaternion.identity, Vector3.one);
    readonly Matrix4x4 m_OrthoMatrix = Matrix4x4.Ortho(-1, 1, -1, 1, 0.01f, 2);
    Material mat2 = null;
    Camera cam = null;
    private void Start()
    {
        cam = GetComponent<Camera>();
        m_CommandBuffer = new CommandBuffer();
        m_CommandBuffer.name = "textBuf";
    }
    void Update()
    {
        if (mesh == null || material == null || other == null)
            return;

        mat2 = new Material(material);
        mat2.shader = shader;
        mat2.SetPass(0);

        //renderTexture.Release();
        //Graphics.SetRenderTarget(renderTexture);
        //Matrix4x4 matrix = other.localToWorldMatrix;

        
        //Graphics.DrawMeshNow(mesh, matrix);

        //Graphics.SetRenderTarget(null);

        m_CommandBuffer.Clear();
        m_CommandBuffer.SetRenderTarget(renderTexture);
        m_CommandBuffer.ClearRenderTarget(true, true, Color.clear);
        m_CommandBuffer.SetViewProjectionMatrices(cam.worldToCameraMatrix, cam.projectionMatrix);
        m_CommandBuffer.DrawMesh(mesh, other.localToWorldMatrix, mat2, 0);
        Graphics.ExecuteCommandBuffer(m_CommandBuffer);
    }
}
