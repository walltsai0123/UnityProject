using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class Test2 : MonoBehaviour
{
    private void Start()
    {
        Vector2Int v1 = new(1, 2);
        Vector2Int v2 = new(1, 2);

        HashSet<Vector2Int> vector2Ints = new HashSet<Vector2Int>();
        vector2Ints.Add(v1);
        vector2Ints.Add(v2);

        Debug.Log(vector2Ints.Count);
        foreach (var v in vector2Ints)
            Debug.Log(v);
    }
}
