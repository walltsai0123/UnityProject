using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using TMPro;
using UnityEngine;
using UnityEngine.UIElements;
//using System.Drawing;
public class NormalMapGenerator : MonoBehaviour
{
    [SerializeField]
    Texture2D heightMap;

    public int dim = 128;
    private int width = 128;
    private int height = 128;
    public float standardDeviationX = 1f;
    public float standardDeviationY = 1f;

    public bool sampleFromHeight = false;
    public float spacing = 1f;
    void Start()
    {
        Vector3[,] normals = new Vector3[dim, dim];
        Texture2D normalMap = null;
        if (sampleFromHeight)
        {
            width = heightMap.width;
            height = heightMap.height;
            normalMap = new Texture2D(width, height);
            Color[] colors = heightMap.GetPixels();
            float[,] heights = new float[width, height];
            normals = new Vector3[width, height];
            for (int w = 0; w < width; w++)
            {
                for (int h = 0; h < height; h++)
                {
                    heights[w, h] = colors[h * width + w].r;
                }
            }

            for (int w = 0; w < width; w++)
            {
                for (int h = 0; h < height; h++)
                {
                    //Sample normal

                    float left = (w == 0) ? heights[w, h] : heights[w - 1, h];
                    float right = (w == width - 1) ? heights[w, h] : heights[w + 1, h];
                    float bottom = (h == 0) ? heights[w, h] : heights[w, h - 1];
                    float top = (h == height - 1) ? heights[w, h] : heights[w, h + 1];

                    float x = -(right - left) / 2f / spacing;
                    float y = -(top - bottom) / 2f / spacing;

                    normals[w, h] = new Vector3(x, y, 1).normalized;

                    Vector3 colorV = normals[w, h] * 0.5f + 0.5f * Vector3.one;
                    Color color = new Color(colorV.x, colorV.y, colorV.z);

                    normalMap.SetPixel(w, h, color);
                }
            }
        }
        else
        {
            width = height = dim;
            normalMap = new Texture2D(width, height);
            for (int w = 0; w < width; w++)
                for (int h = 0; h < height; h++)
                {
                    float nX = GetRandomNormalDistribution(0, standardDeviationX);
                    float nY = GetRandomNormalDistribution(0, standardDeviationY);

                    Vector3 v2 = new(nX, nY, 1f);
                    v2.Normalize();

                    v2 = (v2 + Vector3.one) * 0.5f;
                    Color color = new Color(v2.x, v2.y, v2.z);
                    normalMap.SetPixel(w, h, color);
                }
        }

        normalMap.Apply();
        // Encode texture into PNG
        byte[] normalBytes = ImageConversion.EncodeToPNG(normalMap);
        Object.Destroy(normalMap);

        string fileName = System.DateTime.Now.ToString("MMddyyyy_hh_mm_ss");
        File.WriteAllBytes(Application.dataPath + "/./Shader/Generate Map/normal" + fileName + ".png", normalBytes);
    }
    private float GetRandomNormalDistribution(float mean, float standardDeviation)
    {
        float u1 = Random.Range(0f, 1f);
        float u2 = Random.Range(0f, 1f);
        float z = Mathf.Sqrt(-2f * Mathf.Log(u1)) * Mathf.Sin(2f * Mathf.PI * u2);
        return mean + standardDeviation * z;
    }
    Texture2D Resize(Texture2D original, int newWidth, int newHeight)
    {
        if(original == null) return null;

        Texture2D newTexture = new Texture2D(newWidth, newHeight);
        float scaleX = (float)original.width / newWidth;
        float scaleY = (float)original.height / newHeight;

        for (int x = 0; x < newWidth; x++)
        {
            for (int y = 0; y < newHeight; y++)
            {
                // Bilinear filtering
                float pixelX = x * scaleX;
                float pixelY = y * scaleY;

                // Get the interpolated pixel from the original texture
                Color color = original.GetPixelBilinear(pixelX / original.width, pixelY / original.height);
                newTexture.SetPixel(x, y, color);
            }
        }

        newTexture.Apply();
        return newTexture;
    }
}
