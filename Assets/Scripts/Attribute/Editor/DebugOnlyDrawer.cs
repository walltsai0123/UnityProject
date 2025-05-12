using UnityEditor;
using UnityEngine;

[CustomPropertyDrawer(typeof(DebugOnlyAttribute))]
public class DebugOnlyDrawer : PropertyDrawer
{
    public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
    {
        DebugOnlyAttribute attr = (DebugOnlyAttribute)attribute;

        bool wasEnabled = GUI.enabled;

        if (Application.isPlaying || !attr.AllowEditInEditMode)
            GUI.enabled = false;
        EditorGUI.PropertyField(position, property, label, true);
        GUI.enabled = wasEnabled;
    }
    public override float GetPropertyHeight(SerializedProperty property, GUIContent label)
    {
        return EditorGUI.GetPropertyHeight(property, label, true);
    }
}
