using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(UrdfImportTester))]
public class UrdfPatcherEditor : Editor
{
    private UrdfImportTester urdfImportTester;

    public override void OnInspectorGUI()
    {
        urdfImportTester = (UrdfImportTester)target;

        DrawDefaultInspector();

        GUILayout.Space(10);

        if (GUILayout.Button("Test"))
            urdfImportTester.Test();
    }
}
