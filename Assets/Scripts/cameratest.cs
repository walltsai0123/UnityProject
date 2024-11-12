using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
[RequireComponent(typeof(Camera))]
public class cameratest : MonoBehaviour
{
    public Material material;

    [SerializeField] bool inverse = false;
    [SerializeField, Range(0, Mathf.PI / 2)] float psi;
    [SerializeField] float mu;

    void Start()
    {
        Camera.main.depthTextureMode = DepthTextureMode.DepthNormals;
    }

    private void Update()
    {
        float sqrtTwo = Mathf.Sqrt(2f);
        if (inverse)
        {
            //float temp 
        }
        else
        {
            mu = Mathf.Tan(Mathf.Asin(sqrtTwo * (2 + psi) / (4 * (1 + psi))));
        }
    }

    void OnRenderImage(RenderTexture src, RenderTexture dest)
    {
        if (material != null)
        {
            Graphics.Blit(src, dest, material);
        }
        else
        {
            Graphics.Blit(src, dest);
        }
    }
}
