// TestbedTwin.cs — Unity side of the proximity-operations digital twin.
// Receives joint targets from the Jupyter notebook over ZeroMQ (NetMQ), drives the two
// imported ReBot B601-RS arms, and publishes back what actually happened.
//
//   notebook  --PUB tcp://*:5556  "cmd"  -->  Unity SUB (connect)
//   notebook  <--SUB tcp://*:5557 "state"--   Unity PUB (connect)
//
// Frames: the notebook works in the lab frame (right-handed, Z up, origin = centre of table top).
// Unity is left-handed Y up. Conversion (same as Unity's URDF Importer): unity = (-Y, Z, X), lab = (z, -x, y).
// Joint values: revolute in radians, prismatic (rail) in metres — converted here to the drive units.

using System;
using System.Collections.Generic;
using UnityEngine;
using NetMQ;
using NetMQ.Sockets;

public class TestbedTwin : MonoBehaviour
{
    [Header("Robots imported with the URDF Importer (drag the root GameObjects here)")]
    public Transform chaserRobot;   // ReBot_Chaser_Rail (hangs upside-down from the overhead rail: Rotation (0, 0, 180))
    public Transform debrisRobot;   // ReBot_Debris
    [Tooltip("Optional CAD carriage mesh (BODY2_E-MTHE17…), parked at stroke 0. Parented to rail_carriage on Start so it slides with rail_joint.")]
    public Transform carriageMesh;
    public bool hideUrdfRailBase = true;                     // the CAD actuator replaces the URDF's grey rail_base box

    [Header("ZeroMQ (the notebook binds, Unity connects)")]
    public string cmdAddress = "tcp://localhost:5556";
    public string stateAddress = "tcp://localhost:5557";
    public float stateRateHz = 30f;

    [Header("Test bed")]
    public bool buildTable = true;
    public Vector2 tableSizeXZ = new Vector2(0.74f, 1.39f);  // Unity x = table width, Unity z = 1.39 m length
    public float tcp = 0.05f;                                // satellite centre beyond gripper tip (m)
    public float chaserSize = 0.10f;
    public float debrisSize = 0.10f;
    public float threshold = 0.30f;
    public bool disableRobotColliders = true;                // pure kinematic twin: no self-collision fights

    [Header("Joint drives")]
    public float stiffness = 100000f;
    public float damping = 10000f;
    public float forceLimit = 10000f;

    static readonly string[] ChaserLinks = { "rail_carriage", "link1", "link2", "link3", "link4", "link5", "link6" };
    static readonly string[] DebrisLinks = { "link1", "link2", "link3", "link4", "link5", "link6" };

    [Serializable]
    class Cmd
    {
        public string type; public int seq; public double t;
        public double[] chaser; public double[] debris; public double[] ref_c; public double[] ref_d;
    }
    [Serializable]
    class State
    {
        public string type = "state"; public int seq; public double t; public double sep;
        public double[] chaser_c, debris_c, q_chaser, q_debris, chaser_base, debris_base;
    }

    ArticulationBody[] chJ, deJ;
    Transform chSat, deSat, refMarker, chBase, deBase;
    LineRenderer refPath;
    SubscriberSocket sub;
    PublisherSocket pub;
    Cmd last;
    float nextStateTime, lastRxTime = -999f;
    int lastPathSeq = -1;

    // ---------- frame helpers ----------
    static Vector3 LabToUnity(double[] p) => new Vector3((float)-p[1], (float)p[2], (float)p[0]);
    static double[] UnityToLab(Vector3 u) => new double[] { u.z, -u.x, u.y };

    void Awake()
    {
        // The importer adds a keyboard "Controller" script; it would fight our drives.
        foreach (var robot in new[] { chaserRobot, debrisRobot })
            if (robot != null)
                foreach (var mb in robot.GetComponents<MonoBehaviour>())
                    if (mb.GetType().Name == "Controller") mb.enabled = false;
    }

    void Start()
    {
        if (chaserRobot == null || debrisRobot == null) { Debug.LogError("Assign both robots on TestbedTwin."); enabled = false; return; }

        chJ = FindJoints(chaserRobot, ChaserLinks);
        deJ = FindJoints(debrisRobot, DebrisLinks);
        AttachCarriageMesh();        // before ConfigureRobot, so its colliders are disabled with the robot's
        ConfigureRobot(chaserRobot);
        ConfigureRobot(debrisRobot);
        chBase = FindDeep(chaserRobot, "rail_base") ?? chaserRobot;
        deBase = FindDeep(debrisRobot, "base_link") ?? debrisRobot;
        if (hideUrdfRailBase)
        {
            // only rail_base's own Visuals child: the carriage and the arm are child links of rail_base
            var vis = chBase.Find("Visuals");
            if (vis != null) foreach (var r in vis.GetComponentsInChildren<Renderer>()) r.enabled = false;
        }

        if (buildTable) BuildTable();
        chSat = MakeSatellite(FindDeep(chaserRobot, "gripper_end"), chaserSize, new Color(0.95f, 0.75f, 0.1f), "ChaserSat");
        deSat = MakeSatellite(FindDeep(debrisRobot, "gripper_end"), debrisSize, new Color(0.6f, 0.6f, 0.65f), "DebrisSat");
        var trail = chSat.gameObject.AddComponent<TrailRenderer>();
        trail.time = 10000f; trail.startWidth = trail.endWidth = 0.006f;
        trail.material = new Material(Shader.Find("Sprites/Default")); trail.startColor = trail.endColor = new Color(1f, 0.6f, 0f);

        refMarker = GameObject.CreatePrimitive(PrimitiveType.Sphere).transform;
        refMarker.name = "ReferenceChaser"; refMarker.localScale = Vector3.one * 0.02f;
        Destroy(refMarker.GetComponent<Collider>());
        refMarker.GetComponent<Renderer>().material.color = Color.cyan;
        refPath = new GameObject("ReferencePath").AddComponent<LineRenderer>();
        refPath.startWidth = refPath.endWidth = 0.003f; refPath.positionCount = 0;
        refPath.material = new Material(Shader.Find("Sprites/Default")); refPath.startColor = refPath.endColor = Color.cyan;

        AsyncIO.ForceDotNet.Force();
        sub = new SubscriberSocket();
        sub.Options.ReceiveHighWatermark = 10000;
        sub.Connect(cmdAddress);
        sub.Subscribe("cmd");
        pub = new PublisherSocket();
        pub.Options.SendHighWatermark = 1000;
        pub.Connect(stateAddress);
        Debug.Log($"TestbedTwin: SUB {cmdAddress}, PUB {stateAddress}");
    }

    void Update()
    {
        // Drain everything that arrived; the newest command wins.
        int n = 0;
        while (n < 2000 && sub.TryReceiveFrameString(out string topic))
        {
            string payload = sub.ReceiveFrameString();
            if (topic == "cmd") { last = JsonUtility.FromJson<Cmd>(payload); lastRxTime = Time.time; AppendPath(last); }
            n++;
        }

        if (last != null)
        {
            Apply(chJ, last.chaser);
            Apply(deJ, last.debris);
            if (last.ref_c != null && last.ref_c.Length == 3) refMarker.position = LabToUnity(last.ref_c);
        }

        if (Time.time >= nextStateTime)
        {
            nextStateTime = Time.time + 1f / stateRateHz;
            PublishState();
        }
    }

    void Apply(ArticulationBody[] joints, double[] q)
    {
        if (q == null) return;
        for (int i = 0; i < joints.Length && i < q.Length; i++)
        {
            var j = joints[i]; if (j == null) continue;
            var d = j.xDrive;
            d.target = j.jointType == ArticulationJointType.PrismaticJoint ? (float)q[i] : (float)(q[i] * Mathf.Rad2Deg);
            j.xDrive = d;
        }
    }

    void PublishState()
    {
        var s = new State
        {
            seq = last != null ? last.seq : -1,
            t = last != null ? last.t : 0,
            chaser_c = UnityToLab(chSat.position),
            debris_c = UnityToLab(deSat.position),
            chaser_base = UnityToLab(chBase.position),
            debris_base = UnityToLab(deBase.position),
            q_chaser = ReadJoints(chJ),
            q_debris = ReadJoints(deJ),
            sep = Vector3.Distance(chSat.position, deSat.position)
        };
        pub.SendMoreFrame("state").SendFrame(JsonUtility.ToJson(s));
    }

    static double[] ReadJoints(ArticulationBody[] joints)
    {
        var q = new double[joints.Length];
        for (int i = 0; i < joints.Length; i++)
            q[i] = joints[i] != null && joints[i].dofCount > 0 ? joints[i].jointPosition[0] : double.NaN; // rad or m
        return q;
    }

    void AppendPath(Cmd c)
    {
        if (c.seq < 0 || c.ref_c == null) return;
        if (c.seq < lastPathSeq) refPath.positionCount = 0;           // a new run started
        if (c.seq % 10 == 0)
        {
            refPath.positionCount++;
            refPath.SetPosition(refPath.positionCount - 1, LabToUnity(c.ref_c));
        }
        lastPathSeq = c.seq;
    }

    // ---------- scene helpers ----------
    ArticulationBody[] FindJoints(Transform robot, string[] links)
    {
        var res = new ArticulationBody[links.Length];
        for (int i = 0; i < links.Length; i++)
        {
            var tr = FindDeep(robot, links[i]);
            res[i] = tr != null ? tr.GetComponent<ArticulationBody>() : null;
            if (res[i] == null) Debug.LogError($"{robot.name}: no ArticulationBody on link '{links[i]}'");
        }
        return res;
    }

    void AttachCarriageMesh()
    {
        if (carriageMesh == null) return;
        var carriage = FindDeep(chaserRobot, "rail_carriage");
        if (carriage == null) { Debug.LogError("carriageMesh: no rail_carriage link on the chaser robot"); return; }
        // The mesh is parked at stroke 0, so attach it while the rail is still at 0 (before any command is applied).
        var rail = chJ[0];
        if (rail != null && rail.dofCount > 0 && Mathf.Abs(rail.jointPosition[0]) > 1e-4f)
            Debug.LogWarning($"carriageMesh: rail_joint is at {rail.jointPosition[0]:F4} m, not 0; the mesh will be offset by that much");
        carriageMesh.SetParent(carriage, true);   // keep its world pose
        var vis = carriage.Find("Visuals");       // the URDF's grey carriage box sits inside the CAD carriage
        if (vis != null) foreach (var r in vis.GetComponentsInChildren<Renderer>()) r.enabled = false;
    }

    void ConfigureRobot(Transform robot)
    {
        foreach (var ab in robot.GetComponentsInChildren<ArticulationBody>())
        {
            ab.useGravity = false;
            if (ab.isRoot) ab.immovable = true;
            var d = ab.xDrive; d.stiffness = stiffness; d.damping = damping; d.forceLimit = forceLimit; ab.xDrive = d;
        }
        if (disableRobotColliders)
            foreach (var col in robot.GetComponentsInChildren<Collider>()) col.enabled = false;
    }

    static Transform FindDeep(Transform root, string name)
    {
        if (root.name == name) return root;
        foreach (Transform c in root) { var r = FindDeep(c, name); if (r != null) return r; }
        return null;
    }

    Transform MakeSatellite(Transform tool, float size, Color color, string name)
    {
        var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
        go.name = name; Destroy(go.GetComponent<Collider>());
        go.GetComponent<Renderer>().material.color = color;
        go.transform.SetParent(tool, false);
        go.transform.localPosition = new Vector3(0, 0, tcp);    // URDF +X of gripper_end == Unity local +Z
        go.transform.localRotation = Quaternion.identity;
        go.transform.localScale = Vector3.one * size;
        return go.transform;
    }

    void BuildTable()
    {
        var top = GameObject.CreatePrimitive(PrimitiveType.Cube);
        top.name = "Table"; Destroy(top.GetComponent<Collider>());
        top.transform.position = new Vector3(0, -0.02f, 0);
        top.transform.localScale = new Vector3(tableSizeXZ.x, 0.04f, tableSizeXZ.y);
        top.GetComponent<Renderer>().material.color = new Color(0.82f, 0.78f, 0.70f);
    }

    void OnGUI()
    {
        if (chSat == null) return;
        float sep = Vector3.Distance(chSat.position, deSat.position);
        bool linked = Time.time - lastRxTime < 1f;
        var style = new GUIStyle(GUI.skin.box) { fontSize = 16, alignment = TextAnchor.UpperLeft };
        GUI.Box(new Rect(10, 10, 330, 110),
            $"Link: {(linked ? "receiving" : "waiting for notebook…")}\n" +
            $"t = {(last != null ? last.t : 0):F1} s   seq {(last != null ? last.seq : -1)}\n" +
            $"separation = {sep * 100f:F2} cm\n" +
            $"threshold {threshold * 100f:F0} cm: {(sep <= threshold + 0.002f ? "REACHED" : "not yet")}", style);
    }

    void OnDestroy()
    {
        sub?.Dispose(); pub?.Dispose();
        NetMQConfig.Cleanup(false);   // without this the Editor hangs when you stop Play mode
    }
}

