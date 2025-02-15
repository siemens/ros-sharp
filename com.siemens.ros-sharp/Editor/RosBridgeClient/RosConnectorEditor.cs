#if UNITY_EDITOR

using UnityEditor;
using UnityEditor.Build;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    [CustomEditor(typeof(RosConnector))]
    public class RosConnectorEditor : Editor
    {
        RosConnector rosConnector;

        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();

            // Dropdown to select ROS version
            RosVersion newSelectedRosVersion = (RosVersion)EditorGUILayout.EnumPopup("ROS Version", rosConnector.selectedRosVersion);

            if (newSelectedRosVersion != rosConnector.selectedRosVersion)
            {
                rosConnector.selectedRosVersion = newSelectedRosVersion;
                ToggleROSVersion(rosConnector.selectedRosVersion);
            }
        }

        public void OnEnable()
        {
            rosConnector = (RosConnector)target;
            ToggleROSVersion(rosConnector.selectedRosVersion);
        }

        // Toggle ROS Version
        public static void ToggleROSVersion(RosVersion selectedROSVersion)
        {
            string defineSymbolROS2 = "ROS2"; 

            BuildTargetGroup targetGroup = EditorUserBuildSettings.selectedBuildTargetGroup;
            string defines = PlayerSettings.GetScriptingDefineSymbols(NamedBuildTarget.FromBuildTargetGroup(targetGroup));

            // Remove ROS2 define symbol
            defines = defines.Replace($"{defineSymbolROS2};", "").Replace(defineSymbolROS2, "");

            // Add the define symbol for the selected ROS version (ROS2 is default if not present)
            string defineSymbol = selectedROSVersion == RosVersion.ROS1 ? "" : defineSymbolROS2;
            if (!string.IsNullOrEmpty(defineSymbol) && !defines.Contains(defineSymbol))
            {
                defines += $";{defineSymbol}";
            }

            // Set scripting define symbols
            PlayerSettings.SetScriptingDefineSymbols(NamedBuildTarget.FromBuildTargetGroup(targetGroup), defines);

            // Execute Assembly Builder
            AssetDatabase.Refresh();
        }

        // Display the RosConnector object's inspector
        public static void ShowRosConnectorEditorInspector(GameObject rosConnectorObject)
        {
            EditorGUIUtility.PingObject(rosConnectorObject.gameObject);
            Selection.activeGameObject = rosConnectorObject.gameObject;
            EditorApplication.RepaintHierarchyWindow();
        }
    }
}

#endif