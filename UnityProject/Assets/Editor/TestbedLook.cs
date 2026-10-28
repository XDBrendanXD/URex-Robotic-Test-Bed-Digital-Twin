#if UNITY_EDITOR
// Put this file in Assets/Editor/ and run  Tools > Testbed > Apply CAD Look
// Recreates the CAD file's own materials (from HIL_Main_Robotics.fbx) as URP materials and applies them:
//   Opaque(R,G,B)  -> that exact colour, non-metallic (supplier colours of the foldable table)
//   Steel - Satin  -> Fusion's default steel (160,160,160), satin metal (add-on frame, rail, bumpers, tabletop)
// Matching is by the imported material's name, so it works on any part that came from the FBX.
// Scenery parts whose material name isn't recognised get Steel - Satin. Robots are left alone.
// Safe to run again; Cmd/Ctrl+Z undoes it.
using System.Collections.Generic;
using System.IO;
using System.Text.RegularExpressions;
using UnityEditor;
using UnityEngine;

public static class TestbedLook
{
    const string Dir = "Assets/Materials/Testbed";
    const string Steel = "Steel - Satin";
    static readonly Regex Opaque = new Regex(@"Opaque\((\d+),\s*(\d+),\s*(\d+)\)");
    static readonly Dictionary<string, Material> cache = new Dictionary<string, Material>();

    [MenuItem("Tools/Testbed/Apply CAD Look")]
    internal static void Apply()
    {
        Directory.CreateDirectory(Dir);
        cache.Clear();
        int n = 0;
        foreach (var r in Object.FindObjectsByType<MeshRenderer>(FindObjectsInactive.Include, FindObjectsSortMode.None))
        {
            bool scenery = IsScenery(r.transform);
            var mats = r.sharedMaterials;
            bool changed = false;
            for (int i = 0; i < mats.Length; i++)
            {
                string name = mats[i] != null ? mats[i].name : "";
                Material m = null;
                var mo = Opaque.Match(name);
                if (mo.Success) m = OpaqueMat(int.Parse(mo.Groups[1].Value), int.Parse(mo.Groups[2].Value), int.Parse(mo.Groups[3].Value));
                else if (name.StartsWith(Steel) || scenery) m = SteelMat();
                if (m == null || m == mats[i]) continue;
                mats[i] = m; changed = true;
            }
            if (!changed) continue;
            Undo.RecordObject(r, "Apply CAD look");
            r.sharedMaterials = mats;
            n++;
        }

        var cam = Camera.main;
        if (cam != null)
        {
            Undo.RecordObject(cam, "Apply CAD look");
            cam.clearFlags = CameraClearFlags.SolidColor;
            cam.backgroundColor = new Color32(225, 228, 233, 255);
        }
        AssetDatabase.SaveAssets();
        Debug.Log($"Testbed CAD look applied to {n} renderers ({cache.Count} materials).");
    }

    // Anything inside the imported model, the loose carriage, or the tabletop.
    static bool IsScenery(Transform t)
    {
        if (t.name == "Tabletop" || t.name.StartsWith("BODY2_E-MTHE17") || t.name.StartsWith("CNC Parts")) return true;
        for (; t != null; t = t.parent)
            if (t.name.StartsWith("HIL_Scenery") || t.name.StartsWith("HIL_Table") || t.name.StartsWith("HIL_Main_Robotics")) return true;
        return false;
    }

    // "Opaque" appearances in the CAD: plain coloured, Shininess 20 -> non-metal, low gloss.
    static Material OpaqueMat(int r, int g, int b) =>
        Mat($"Opaque_{r}_{g}_{b}", new Color32((byte)r, (byte)g, (byte)b, 255), 0f, 0.3f);

    // Fusion "Steel - Satin": diffuse 0.627 (160,160,160), brushed metal.
    static Material SteelMat() =>
        Mat("Steel_Satin", new Color32(160, 160, 160, 255), 0.7f, 0.5f);

    static Material Mat(string name, Color c, float metallic, float smoothness)
    {
        if (cache.TryGetValue(name, out var cached)) return cached;
        string p = $"{Dir}/{name}.mat";
        var m = AssetDatabase.LoadAssetAtPath<Material>(p);
        if (m == null)
        {
            m = new Material(Shader.Find("Universal Render Pipeline/Lit"));
            AssetDatabase.CreateAsset(m, p);
        }
        m.SetColor("_BaseColor", c);
        m.SetFloat("_Metallic", metallic);
        m.SetFloat("_Smoothness", smoothness);
        EditorUtility.SetDirty(m);
        cache[name] = m;
        return m;
    }
}
#endif
