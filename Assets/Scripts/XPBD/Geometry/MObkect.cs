using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[CreateAssetMenu(fileName ="MyObject", menuName ="CreateMyObject")]
public class MObkect : ScriptableObject
{
    public string Name;
    public float[] atk;
    public float price;
    public Sprite sp;
}
