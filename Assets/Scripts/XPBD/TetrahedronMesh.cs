using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[CreateAssetMenu(fileName = "NewTetrahedronMesh", menuName = "TetrahedronMesh")]
public class TetrahedronMesh : ScriptableObject
{
    public Vector3[] vertices;
    public int[] faces;
    public int[] tets;
    public int[] edges;
}
