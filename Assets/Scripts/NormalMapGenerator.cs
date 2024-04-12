using System.Collections;
using System.Collections.Generic;
using System.IO;
using TMPro;
using UnityEngine;
using UnityEngine.UIElements;

public class NormalMapGenerator : MonoBehaviour
{
    [SerializeField]
    //Texture2D normalMap;

    public int dim = 128;
    private int width = 128;
    private int height = 128;
    public float standardDeviationX = 1f;
    public float standardDeviationY = 1f;
    public string fileName = "newTex";
    // Start is called before the first frame update
    void Start()
    {
        width = height = dim;
        Texture2D newTex = new Texture2D(width, height);
        for(int w = 0; w < width; w++)
            for(int h = 0; h < height; h++)
            {
                float nx = Random.Range(0, standardDeviationX);
                float ny = Random.Range(0, standardDeviationY);
                Vector3 v = new Vector3(nx * 2f - 1f, ny * 2f - 1f, 1f).normalized;

                v = (v + Vector3.one) * 0.5f;
                Color normal = new Color(v.x, v.y, v.z);
                //SetPixel(w, h, normal);


                float nX = GetRandomNormalDistribution(0, standardDeviationX);
                float nY = GetRandomNormalDistribution(0, standardDeviationY);
                Vector3 v2 = new(nX, nY, 1f);
                v2.Normalize();

                v2 = (v2 + Vector3.one) * 0.5f;
                Color color = new Color(v2.x, v2.y, v2.z);
                newTex.SetPixel(w, h, color);
            }

        //Debug.Log(normalMap.GetPixel(0,0));

        //normalMap.Apply();
        // Encode texture into PNG
        //byte[] bytes = ImageConversion.EncodeToPNG(normalMap);
        ////Object.Destroy(normalMap);

        //// For testing purposes, also write to a file in the project folder
        //File.WriteAllBytes(Application.dataPath + "/./Shader/texture.png", bytes);

        newTex.Apply();
        // Encode texture into PNG
        byte[] bytes2 = ImageConversion.EncodeToPNG(newTex);
        Object.Destroy(newTex);

        // For testing purposes, also write to a file in the project folder
        if (fileName.Length < 1)
            fileName = "newTexture";
        File.WriteAllBytes(Application.dataPath + "/./Shader/" + fileName + ".png", bytes2);
    }
    private float GetRandomNormalDistribution(float mean, float standardDeviation)
    {
        float u1 = Random.Range(0f, 1f);
        float u2 = Random.Range(0f, 1f);
        float z = Mathf.Sqrt(-2f * Mathf.Log(u1)) * Mathf.Sin(2f * Mathf.PI * u2);
        return mean + standardDeviation * z;
    }
}
