using UnityEngine;

public class DebugOnlyAttribute : PropertyAttribute
{
    public bool AllowEditInEditMode { get; private set; }

    public DebugOnlyAttribute(bool allowEditInEditMode = false)
    {
        AllowEditInEditMode = allowEditInEditMode;
    }
}
