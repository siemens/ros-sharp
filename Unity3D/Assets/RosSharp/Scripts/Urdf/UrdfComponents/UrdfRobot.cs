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

using System;
using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEngine;

namespace RosSharp.Urdf.Export
{
    public class UrdfRobot : MonoBehaviour
    {
        private string filePath;

        public void ExportRobotToUrdf(string robotAssetFolder)
        {
            UrdfAssetPathHandler.SetAssetRootFolder(robotAssetFolder);
            filePath = Path.Combine(robotAssetFolder, name + ".urdf");
            filePath = filePath.Replace(Path.AltDirectorySeparatorChar, Path.DirectorySeparatorChar);

            string meshRootFolder = EditorUtility.OpenFolderPanel(
                "Select mesh export location",
                UrdfAssetPathHandler.GetAssetRootFolder(),
                "");
            try
            {
                meshRootFolder.Substring(UrdfAssetPathHandler.GetAssetRootFolder().Length);
            }
            catch (Exception e)
            {
                Debug.LogWarning("Mesh export folder must be a subfolder of the robot export location. Aborting URDF export. " + e);
                return;
            }

            UrdfAssetPathHandler.SetMeshRootFolder(meshRootFolder);

            Robot robot = GetRobotData();

            if (robot == null) return;

            robot.WriteToUrdf();
            AssetDatabase.Refresh();

            Debug.Log(robot.name + " was exported to " + UrdfAssetPathHandler.GetRelativeAssetPath(filePath));
        }

        public void Reset()
        {
            transform.DestroyChildrenImmediate();

            filePath = Application.dataPath + Path.DirectorySeparatorChar + "Urdf" + Path.DirectorySeparatorChar + gameObject.name + ".urdf";

            GameObject baseLink = new GameObject("base_link");
            baseLink.transform.SetParentAndAlign(gameObject.transform);

            baseLink.AddComponent<UrdfLink>();

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
                
            
            //TODO find and save all materials/meshes in the urdf folder, and add to robot

            //foreach(Material material in gameObject.GetComponentsInChildren<Material>())
            //    robot.materials.Add(GetMaterialData(material));

            return robot;
        }


    }
}
