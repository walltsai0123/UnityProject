using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public abstract class Geometry : MonoBehaviour
{
    public abstract bool Raycast(Ray ray);
}
