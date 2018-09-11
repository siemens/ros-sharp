/*
© Siemens AG, 2018
Author: Suzannah Smith (suzannah.smith@siemens.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

<http://www.apache.org/licenses/LICENSE-2.0>.

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using System.Collections.Generic;
using System.IO;
using System.Linq;
using RosSharp.RosBridgeClient;
using UnityEditor;
using UnityEngine;

namespace RosSharp.Urdf
{
    public class UrdfRobot : MonoBehaviour
    {
        private string filePath;

        public static void Create()
        {
            GameObject robotGameObject = new GameObject("Robot");
            robotGameObject.AddComponent<UrdfRobot>();
            robotGameObject.AddComponent<RobotConfigurator>();

            UrdfLink urdfLink = UrdfLink.Create(robotGameObject.transform);
            urdfLink.name = "base_link";
        }

        public static UrdfRobot Create(string filename)
        {
            Robot robot = new Robot(filename);

            if (!UrdfAssetPathHandler.IsValidAssetPath(robot.filename))
            {
                Debug.LogError("URDF file and ressources must be placed in Assets Folder:\n" + Application.dataPath);
                return null;
            }

            GameObject robotGameObject = new GameObject(robot.name);
            UrdfRobot urdfRobot = robotGameObject.AddComponent<UrdfRobot>();
            robotGameObject.AddComponent<RobotConfigurator>();

            UrdfAssetPathHandler.SetPackageRoot(Path.GetDirectoryName(robot.filename));
            UrdfMaterialHandler.InitializeRobotMaterials(robot);

            UrdfLink.Create(robotGameObject.transform, robot.root);

            GameObjectUtility.SetParentAndAlign(robotGameObject, Selection.activeObject as GameObject);
            Undo.RegisterCreatedObjectUndo(robotGameObject, "Create " + robotGameObject.name);
            Selection.activeObject = robotGameObject;
            
            return urdfRobot;
        }

        private void GenerateUniqueJointNames()
        {
            foreach (UrdfJoint urdfJoint in GetComponentsInChildren<UrdfJoint>())
                urdfJoint.GenerateUniqueJointName();
        }

        public void ExportRobotToUrdf(string exportRootFolder, string exportDestination)
        {
            UrdfExportPathHandler.SetExportPath(exportRootFolder, exportDestination);
            GenerateUniqueJointNames();

            filePath = Path.Combine(UrdfExportPathHandler.GetExportDestination(), name + ".urdf");
            filePath = filePath.Replace(Path.AltDirectorySeparatorChar, Path.DirectorySeparatorChar);
    
            Robot robot = GetRobotData();
            if (robot == null) return;

            robot.WriteToUrdf();

            Debug.Log(robot.name + " was exported to " + UrdfExportPathHandler.GetExportDestination());

            UrdfMaterial.materials.Clear();
            UrdfExportPathHandler.Clear();
            AssetDatabase.Refresh();
        }
        
        private Robot GetRobotData()
        {
            Robot robot = new Robot(filePath, gameObject.name);

            List<string> linkNames = new List<string>();
            foreach (UrdfLink urdfLink in gameObject.GetComponentsInChildren<UrdfLink>())
            {
                if (linkNames.Contains(urdfLink.name))
                {
                    EditorUtility.DisplayDialog("URDF Export Error",
                        "URDF export failed. There are several links with the name " +
                        urdfLink.name + ". Make sure all link names are unique before exporting this robot.",
                        "Ok");
                    return null;
                }
                robot.links.Add(urdfLink.GetLinkData());
                linkNames.Add(urdfLink.name);
            }

            List<string> jointNames = new List<string>();
            foreach (UrdfJoint urdfJoint in gameObject.GetComponentsInChildren<UrdfJoint>())
            {
                Joint joint = urdfJoint.GetJointData();
                if (jointNames.Contains(urdfJoint.JointName))
                {
                    EditorUtility.DisplayDialog("URDF Export Error",
                        "URDF export failed. There is more than one joint with the name " +
                        urdfJoint.JointName + ". Make sure all joint names are unique before exporting this robot.",
                        "Ok");
                    Debug.LogError("URDF export failed.");
                    return null;
                }
                jointNames.Add(urdfJoint.JointName);
                if (joint != null) robot.joints.Add(joint);
            }

            robot.materials = UrdfMaterial.materials.Values.ToList();

            return robot;
        }
    }
}
