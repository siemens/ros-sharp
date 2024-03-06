using UnityEditor;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    [CustomEditor(typeof(RosConnector))]
    public class RosConnectorEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();

            RosConnector rosConnector = (RosConnector)target;

            // Dropdown to select ROS version
            ROSVersion newSelectedROSVersion = (ROSVersion)EditorGUILayout.EnumPopup("ROS Version", rosConnector.selectedROSVersion);

            if (newSelectedROSVersion != rosConnector.selectedROSVersion)
            {
                rosConnector.selectedROSVersion = newSelectedROSVersion;

                // Combine all actions when the dropdown changes:
                //  If different scenes in a single project works with different ROS versions, this
                // function needs to be called again. Switching scenes does not switch ROS version!
                ToggleROSVersion(rosConnector.selectedROSVersion);
            }
        }

        // Toggle ROS Version
        private static void ToggleROSVersion(ROSVersion selectedROSVersion)
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
