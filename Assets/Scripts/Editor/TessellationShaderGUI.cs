using UnityEngine;
using UnityEditor;

public class TessellationShaderGUI : ShaderGUI
{
    enum TessellationMode
    {
        Uniform, Edge
    }
    enum SmoothnessSource
    {
        Uniform, Albedo, Metallic
    }

    static GUIContent staticLabel = new GUIContent();

    Material target;
    MaterialEditor editor;
    MaterialProperty[] properties;
    public override void OnGUI(MaterialEditor materialEditor, MaterialProperty[] properties)
    {
        //base.OnGUI(materialEditor, properties);
        // get the current keywords from the material
        target = materialEditor.target as Material;
        editor = materialEditor;
        this.properties = properties;

        DoTessellation();
        DoMain();
    }

    void DoTessellation()
    {
        GUILayout.Label("Tessellation", EditorStyles.boldLabel);
        EditorGUI.indentLevel += 2;
        TessellationMode mode = TessellationMode.Uniform;
        if (IsKeywordEnabled("_TESSELLATION_EDGE"))
        {
            mode = TessellationMode.Edge;
        }
        EditorGUI.BeginChangeCheck();
        mode = (TessellationMode)EditorGUILayout.EnumPopup(
            MakeLabel("Mode"), mode
        );
        if (EditorGUI.EndChangeCheck())
        {
            RecordAction("Tessellation Mode");
            SetKeyword("_TESSELLATION_EDGE", mode == TessellationMode.Edge);
        }

        if (mode == TessellationMode.Uniform)
        {
            editor.ShaderProperty(
                FindProperty("_TessellationUniform"),
                MakeLabel("Uniform")
            );
        }
        else
        {
            editor.ShaderProperty(
                FindProperty("_TessellationEdgeLength"),
                MakeLabel("Edge Length")
            );
        }
        EditorGUI.indentLevel -= 2;
    }

    void DoMain()
    {
        GUILayout.Label("Main Maps", EditorStyles.boldLabel);

        MaterialProperty mainTex = FindProperty("_MainTex");
        editor.TexturePropertySingleLine(
            MakeLabel(mainTex, "Albedo (RGB)"), mainTex, FindProperty("_Color")
        );
        DoMetallic();
        DoSmoothness();
        DoNormals();
        //DoParallax();
        //DoOcclusion();
        //DoEmission();
        //DoDetailMask();
        editor.TextureScaleOffsetProperty(mainTex);
    }
    void DoMetallic()
    {
        MaterialProperty map = FindProperty("_MetallicMap");
        Texture tex = map.textureValue;
        EditorGUI.BeginChangeCheck();
        editor.TexturePropertySingleLine(
            MakeLabel(map, "Metallic (R)"), map,
            tex ? null : FindProperty("_Metallic")
        );
        if (EditorGUI.EndChangeCheck() && tex != map.textureValue)
        {
            SetKeyword("_METALLIC_MAP", map.textureValue);
        }
    }

    void DoSmoothness()
    {
        SmoothnessSource source = SmoothnessSource.Uniform;
        if (IsKeywordEnabled("_SMOOTHNESS_ALBEDO"))
        {
            source = SmoothnessSource.Albedo;
        }
        else if (IsKeywordEnabled("_SMOOTHNESS_METALLIC"))
        {
            source = SmoothnessSource.Metallic;
        }
        MaterialProperty slider = FindProperty("_Smoothness");
        EditorGUI.indentLevel += 2;
        editor.ShaderProperty(slider, MakeLabel(slider));
        EditorGUI.indentLevel += 1;
        EditorGUI.BeginChangeCheck();
        source = (SmoothnessSource)EditorGUILayout.EnumPopup(
            MakeLabel("Source"), source
        );
        if (EditorGUI.EndChangeCheck())
        {
            RecordAction("Smoothness Source");
            SetKeyword("_SMOOTHNESS_ALBEDO", source == SmoothnessSource.Albedo);
            SetKeyword(
                "_SMOOTHNESS_METALLIC", source == SmoothnessSource.Metallic
            );
        }
        EditorGUI.indentLevel -= 3;
    }
    void DoNormals()
    {
        MaterialProperty map = FindProperty("_BumpMap");
        Texture tex = map.textureValue;
        EditorGUI.BeginChangeCheck();
        editor.TexturePropertySingleLine(
            MakeLabel(map), map,
            tex ? FindProperty("_BumpScale") : null
        );
        if (EditorGUI.EndChangeCheck() && tex != map.textureValue)
        {
            SetKeyword("_NORMAL_MAP", map.textureValue);
        }
    }


    MaterialProperty FindProperty(string name)
    {
        return FindProperty(name, properties);
    }

    static GUIContent MakeLabel(string text, string tooltip = null)
    {
        staticLabel.text = text;
        staticLabel.tooltip = tooltip;
        return staticLabel;
    }

    static GUIContent MakeLabel(
        MaterialProperty property, string tooltip = null
    )
    {
        staticLabel.text = property.displayName;
        staticLabel.tooltip = tooltip;
        return staticLabel;
    }

    void SetKeyword(string keyword, bool state)
    {
        if (state)
        {
            foreach (Material m in editor.targets)
            {
                m.EnableKeyword(keyword);
            }
        }
        else
        {
            foreach (Material m in editor.targets)
            {
                m.DisableKeyword(keyword);
            }
        }
    }

    bool IsKeywordEnabled(string keyword)
    {
        return target.IsKeywordEnabled(keyword);
    }

    void RecordAction(string label)
    {
        editor.RegisterPropertyChangeUndo(label);
    }
}
