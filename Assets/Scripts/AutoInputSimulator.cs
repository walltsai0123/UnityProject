using UnityEngine;

public class AutoInputSimulator : MonoBehaviour
{
    public float simulatedVertical = 0f;
    public int startFrame = 600;
    public int durationFrames = 600;
    private int currentFrame = 0;

    void Update()
    {
        currentFrame++;

        // 模擬 Vertical Input 為 1，持續 600 frame
        if (currentFrame >= startFrame && currentFrame < startFrame + durationFrames)
        {
            simulatedVertical = 1f;
        }
        else
        {
            simulatedVertical = 0f;
        }

        // 用這個值來控制物件，等於取代 Input.GetAxis("Vertical")
        //transform.Translate(Vector3.forward * simulatedVertical * Time.deltaTime * 5f);
    }
}
