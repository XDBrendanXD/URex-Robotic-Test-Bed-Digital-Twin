#if UNITY_EDITOR
// Put this file in Assets/Editor/ and run  Tools > Testbed > Set Up HIL Scenery
// Prepares the imported CAD export (HIL_Main_Robotics.fbx) for the twin, steps 3–6 of the v3 handoff:
//   - deletes the CAD arms (1-*.step, 2-*.step, RS00*, RS06*, reBot_B601*): the URDF robots replace them
//   - hides the cable chain (b15i*): it is static and would not follow the carriage
//   - parks the carriage group (BODY2_E-MTHE17… + CNC Parts:2) at stroke 0, lab x = -0.795
//   - centres the debris adapter plate (CNC Parts:1) under the debris base, lab (0.60, 0.23), top face at 0.010
//   - assigns the carriage group to TestbedTwin.carriageMeshes and unticks Build Table
// Positions are set from the renderer bounds (absolute, not relative), so it is safe to run again; Cmd/Ctrl+Z undoes it.
// Select the FBX instance first, or it uses the first root named HIL_Main_Robotics*. Values must match proximity_ops_twin.py.
using System.Linq;
using System.Text.RegularExpressions;
using UnityEditor;
using UnityEngine;

public static class TestbedSceneSetup
{
    const float CarriageStroke0Z = -0.795f;                        // Unity z = lab x of the carriage centre at stroke 0
    static readonly Vector2 DebrisPlateXZ = new Vector2(-0.23f, 0.60f);  // Unity (x, z) = lab (-y, x) of the debris base
    const float DebrisPlateTopY = 0.010f;                          // = DEBRIS_BASE_Z
    const string FbxPath = "Assets/Table/HIL_Main_Robotics.fbx";
    static readonly Regex ArmPart = new Regex(@"^([12]-.*\.step|RS00|RS06|reBot_B601)");

    [MenuItem("Tools/Testbed/Set Up HIL Scenery")]
    static void Run()
    {
        var root = FindRoot();
        if (root == null) { Debug.LogError("Set Up HIL Scenery: select the imported HIL_Main_Robotics object first."); return; }
        if (PrefabUtility.IsPartOfPrefabInstance(root))
            PrefabUtility.UnpackPrefabInstance(PrefabUtility.GetOutermostPrefabInstanceRoot(root),
                                               PrefabUnpackMode.Completely, InteractionMode.UserAction);
        var all = root.GetComponentsInChildren<Transform>(true);

        // CAD arms: delete only the topmost matches (their children go with them). Carriage, rail and plates that the
        // assembly nests inside an arm part are moved out to the root first, keeping their world pose.
        var arms = all.Where(t => ArmPart.IsMatch(t.name) && !HasMatchingParent(t, root)).ToList();
        foreach (var t in all.Where(t => (t.name.Contains("MTHE17") || t.name.StartsWith("CNC Parts:")) && arms.Any(a => t.IsChildOf(a))).ToList())
        {
            Debug.Log($"Set Up HIL Scenery: moved {Path(t, root)} out of the CAD arm before deleting it");
            Undo.SetTransformParent(t, root, true, "Set up HIL scenery");
        }
        foreach (var t in arms) Undo.DestroyObjectImmediate(t.gameObject);
        all = root.GetComponentsInChildren<Transform>(true);
        foreach (var t in all.Where(t => t.name.Contains("MTHE17") || t.name.StartsWith("CNC Parts:")))
            Debug.Log($"Set Up HIL Scenery: found {Path(t, root)}");

        int chain = 0;
        foreach (var t in all.Where(t => t.name.StartsWith("b15i") && t.gameObject.activeSelf))
        { Undo.RecordObject(t.gameObject, "Set up HIL scenery"); t.gameObject.SetActive(false); chain++; }

        var body = all.FirstOrDefault(t => t.name.StartsWith("BODY2_E-MTHE17"));
        var carriagePlate = all.FirstOrDefault(t => t.name == "CNC Parts:2");
        var debrisPlate = all.FirstOrDefault(t => t.name == "CNC Parts:1");
        if (body == null || carriagePlate == null || debrisPlate == null)
        { Debug.LogError($"Set Up HIL Scenery: missing BODY2_E-MTHE17… ({body != null}), CNC Parts:2 ({carriagePlate != null}) or CNC Parts:1 ({debrisPlate != null})"); return; }

        // Carriage group: shift both by the same amount so the carriage centre sits at stroke 0
        float dz = CarriageStroke0Z - Bounds(body).center.z;
        foreach (var t in new[] { body, carriagePlate })
        { Undo.RecordObject(t, "Set up HIL scenery"); t.position += new Vector3(0, 0, dz); }

        // Debris plate: centred under the debris base, top face at DEBRIS_BASE_Z
        var b = Bounds(debrisPlate);
        Undo.RecordObject(debrisPlate, "Set up HIL scenery");
        debrisPlate.position += new Vector3(DebrisPlateXZ.x - b.center.x, DebrisPlateTopY - b.max.y, DebrisPlateXZ.y - b.center.z);

        var twin = Object.FindAnyObjectByType<TestbedTwin>(FindObjectsInactive.Include);
        if (twin != null)
        {
            Undo.RecordObject(twin, "Set up HIL scenery");
            twin.carriageMeshes = new[] { body, carriagePlate };
            twin.buildTable = false;
            EditorUtility.SetDirty(twin);
        }

        foreach (var name in new[] { "HIL_Table", "HIL_Scenery", "Tabletop" })
        {
            var old = GameObject.Find(name);
            if (old != null && !old.transform.IsChildOf(root)) Debug.LogWarning($"Set Up HIL Scenery: old '{name}' is still in the scene; delete it.");
        }
        Debug.Log($"HIL scenery set up: deleted {arms.Count} CAD arm objects, hid {chain} cable-chain objects, " +
                  $"moved the carriage group by {dz:F3} m along Z, debris plate centred at ({DebrisPlateXZ.x}, {DebrisPlateTopY}, {DebrisPlateXZ.y})" +
                  (twin != null ? ", TestbedTwin.carriageMeshes assigned." : ". No TestbedTwin found: assign carriageMeshes by hand."));
    }

    // Everything for the v3 layout in one go: the scenery steps above, the old scenery removed, the robots placed
    // (values from the section 5b printout of proximity_ops_twin.py) and the CAD look applied. Save the scene afterwards.
    [MenuItem("Tools/Testbed/Apply v3 Layout")]
    static void ApplyV3()
    {
        Undo.SetCurrentGroupName("Apply v3 layout");
        var root = Rebuild();
        if (root == null) return;

        // old export and placeholder table (exact names; the new FBX's own parts are kept)
        foreach (var t in Object.FindObjectsByType<Transform>(FindObjectsInactive.Include)
                                .Where(t => t != null && (t.name == "HIL_Table" || t.name == "HIL_Scenery" || t.name == "Tabletop")
                                            && !t.IsChildOf(root) && !root.IsChildOf(t)).ToList())
        { Debug.Log($"Apply v3 Layout: deleted old '{t.name}'"); Undo.DestroyObjectImmediate(t.gameObject); }

        Run();
        Place("ReBot_Chaser_Rail", new Vector3(0f, 0.943f, -0.795f), new Vector3(0, 0, 180));
        Place("ReBot_Debris", new Vector3(-0.230f, 0.010f, 0.600f), new Vector3(0, 90, 0));
        TestbedLook.Apply();
        UnityEditor.SceneManagement.EditorSceneManager.MarkSceneDirty(root.gameObject.scene);
        Report(root);
    }

    // Replaces every HIL_Main_Robotics instance with a fresh one from the FBX under TestBed_Scenery, then moves it so the
    // tabletop's top-centre is at the Unity origin (= lab origin). The export's own origin is not the table centre.
    static Transform Rebuild()
    {
        var asset = AssetDatabase.LoadAssetAtPath<GameObject>(FbxPath);
        if (asset == null) { Debug.LogError($"Apply v3 Layout: {FbxPath} not found"); return null; }
        var olds = Object.FindObjectsByType<Transform>(FindObjectsInactive.Include).Where(t => t.name.StartsWith("HIL_Main_Robotics")).ToList();
        var parent = olds.Select(t => t.parent).FirstOrDefault(p => p != null) ?? GameObject.Find("TestBed_Scenery")?.transform;
        if (parent == null) { Debug.LogError("Apply v3 Layout: no TestBed_Scenery object in the scene"); return null; }
        foreach (var t in olds) if (t != null) Undo.DestroyObjectImmediate(t.gameObject);

        var go = (GameObject)PrefabUtility.InstantiatePrefab(asset, parent);
        Undo.RegisterCreatedObjectUndo(go, "Apply v3 layout");
        PrefabUtility.UnpackPrefabInstance(go, PrefabUnpackMode.Completely, InteractionMode.AutomatedAction);
        var root = go.transform;
        root.localPosition = Vector3.zero; root.localRotation = Quaternion.identity; root.localScale = Vector3.one;

        var top = root.GetComponentsInChildren<Transform>(true).FirstOrDefault(t => t.name.StartsWith("Tabletop"));
        if (top == null) { Debug.LogError("Apply v3 Layout: no Tabletop in the FBX"); return root; }
        var b = Bounds(top);
        root.position -= new Vector3(b.center.x, b.max.y, b.center.z);
        Debug.Log($"Apply v3 Layout: fresh {root.name} under {parent.name}, shifted by {(-new Vector3(b.center.x, b.max.y, b.center.z)).ToString("F4")} " +
                  $"to put the tabletop top-centre at the origin (tabletop {b.size.ToString("F3")})");
        Selection.activeTransform = root;
        return root;
    }

    static string Path(Transform t, Transform root)
    {
        var s = t.name;
        for (var p = t.parent; p != null && p != root; p = p.parent) s = p.name + "/" + s;
        return s;
    }

    static void Place(string name, Vector3 pos, Vector3 euler)
    {
        var go = GameObject.Find(name);
        if (go == null) { Debug.LogError($"Apply v3 Layout: no '{name}' in the scene"); return; }
        Undo.RecordObject(go.transform, "Apply v3 layout");
        go.transform.SetPositionAndRotation(pos, Quaternion.Euler(euler));
    }

    // Handoff step 2 checks, written to the log: tabletop top at y = 0, rail along Unity Z, drawer box at -Z.
    static void Report(Transform root)
    {
        var all = root.GetComponentsInChildren<Transform>(true);
        string B(string prefix)
        {
            var t = all.FirstOrDefault(x => x.name.StartsWith(prefix));
            if (t == null) return $"{prefix}: not found";
            var b = Bounds(t);
            return $"{prefix}: min {b.min.ToString("F3")} max {b.max.ToString("F3")}";
        }
        var p = root.parent;
        Debug.Log("Apply v3 Layout report\n" +
                  $"scenery parent {(p != null ? p.name : "-")}: pos {(p != null ? p.position.ToString("F3") : "-")} rot {(p != null ? p.eulerAngles.ToString("F1") : "-")}\n" +
                  string.Join("\n", new[] { "BODY1", "BODY2_E-MTHE17", "CNC Parts:2", "CNC Parts:1", "Tabletop", "FOLDABLE" }.Select(B)) + "\n" +
                  string.Join("\n", new[] { "ReBot_Chaser_Rail", "ReBot_Debris" }.Select(n =>
                  { var g = GameObject.Find(n); return g == null ? $"{n}: not found" : $"{n}: pos {g.transform.position.ToString("F3")} rot {g.transform.eulerAngles.ToString("F1")}"; })));
    }

    // Lets an external tool run ApplyV3 once: create <project>/Temp/testbed_apply_v3 and focus the Editor.
    [InitializeOnLoadMethod]
    static void WatchTrigger() => EditorApplication.delayCall += () =>
    {
        const string trigger = "Temp/testbed_apply_v3";
        if (!System.IO.File.Exists(trigger) || EditorApplication.isPlayingOrWillChangePlaymode) return;
        System.IO.File.Delete(trigger);
        ApplyV3();
    };

    static Transform FindRoot()
    {
        for (var t = Selection.activeTransform; t != null; t = t.parent)
            if (t.name.StartsWith("HIL_Main_Robotics")) return t;
        return Object.FindObjectsByType<Transform>(FindObjectsInactive.Include)
                     .FirstOrDefault(t => t.name.StartsWith("HIL_Main_Robotics"));
    }

    static bool HasMatchingParent(Transform t, Transform root)
    {
        for (var p = t.parent; p != null && p != root; p = p.parent)
            if (ArmPart.IsMatch(p.name)) return true;
        return false;
    }

    static Bounds Bounds(Transform t)
    {
        var rs = t.GetComponentsInChildren<Renderer>(true);
        if (rs.Length == 0) return new Bounds(t.position, Vector3.zero);
        var b = rs[0].bounds;
        foreach (var r in rs) b.Encapsulate(r.bounds);
        return b;
    }
}
#endif
