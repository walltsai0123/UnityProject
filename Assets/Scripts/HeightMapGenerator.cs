using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Xml;
using UnityEngine;

public class HeightMapGenerator : MonoBehaviour
{

    public int dim = 128;
    public float standardDeviation = 1f;
    readonly float baseDeviation = 0.16667f;
    // Start is called before the first frame update
    void Start()
    {
        //float[,] heights = new float[dim, dim];
        Texture2D heightMap = new Texture2D(dim, dim);

        int width = dim;
        int height = dim;

        for (int w = 0; w < width; w++)
        {
            for (int h = 0; h < height; h++)
            {
                //float nX = GetRandomNormalDistribution(0.5f, baseDeviation * standardDeviation);
                //heights[w, h] = nX;
                int subHeight = height / 4;

                float nX = (float)(h % subHeight) / subHeight;
                Color color = new(nX, nX, nX);

                heightMap.SetPixel(w, h, color);
            }
        }

        heightMap.Apply();
        // Encode texture into PNG
        byte[] heightBytes = ImageConversion.EncodeToPNG(heightMap);
        Object.Destroy(heightMap);

        string fileName = System.DateTime.Now.ToString("MMddyyyy_hh_mm_ss");
        File.WriteAllBytes(Application.dataPath + "/./Shader/Generate Map/height" + fileName + ".png", heightBytes);
    }
    private float GetRandomNormalDistribution(float mean, float standardDeviation)
    {
        float u1 = Random.Range(0f, 1f);
        float u2 = Random.Range(0f, 1f);
        float z = Mathf.Sqrt(-2f * Mathf.Log(u1)) * Mathf.Sin(2f * Mathf.PI * u2);
        return mean + standardDeviation * z;
    }
}
