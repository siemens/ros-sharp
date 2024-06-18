using UnityEditor;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    [CustomEditor(typeof(RosConnector))]
    public class RosConnectorEditor : Editor
    {
        private static bool isInitialized = false;
        public static bool IsInitialized { get; set; }

        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();

            RosConnector rosConnector = (RosConnector)target;

            // Dropdown to select ROS version
            ROSVersion newSelectedROSVersion = (ROSVersion)EditorGUILayout.EnumPopup("ROS Version", rosConnector.selectedROSVersion);

            if (!isInitialized || newSelectedROSVersion != rosConnector.selectedROSVersion)
            {
                rosConnector.selectedROSVersion = newSelectedROSVersion;
                ToggleROSVersion(rosConnector.selectedROSVersion);
                isInitialized = true;
            }
        }

        // Toggle ROS Version
        public static void ToggleROSVersion(ROSVersion selectedROSVersion)
        {
            string defineSymbolROS2 = "ROS2"; 

            BuildTargetGroup targetGroup = EditorUserBuildSettings.selectedBuildTargetGroup;
            string defines = PlayerSettings.GetScriptingDefineSymbolsForGroup(targetGroup);

            // Remove ROS2 define symbol
            defines = defines.Replace($"{defineSymbolROS2};", "").Replace(defineSymbolROS2, "");

            // Add the define symbol for the selected ROS version (ROS2 is default if not present)
            string defineSymbol = selectedROSVersion == ROSVersion.ROS1 ? "" : defineSymbolROS2;
            if (!string.IsNullOrEmpty(defineSymbol) && !defines.Contains(defineSymbol))
            {
                defines += $";{defineSymbol}";
            }

            // Set scripting define symbols
            PlayerSettings.SetScriptingDefineSymbolsForGroup(targetGroup, defines);

            // Execute Assembly Builder
            AssetDatabase.Refresh();
        }
    }
}
