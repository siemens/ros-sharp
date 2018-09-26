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
using UnityEditor;
using UnityEngine;

namespace RosSharp.Urdf.Editor
{
    public static class UrdfRobotExtensions
    {
        public static void Create()
        {
            GameObject robotGameObject = new GameObject("Robot");
            robotGameObject.AddComponent<UrdfRobot>();

            UrdfPluginsExtensions.Create(robotGameObject.transform);

            UrdfLink urdfLink = UrdfLinkExtensions.Create(robotGameObject.transform);
            urdfLink.name = "base_link";
            urdfLink.IsBaseLink = true;
        }

        #region Import

        public static void Create(string filename)
        {
            Robot robot = new Robot(filename);

            if (!UrdfAssetPathHandler.IsValidAssetPath(robot.filename))
            {
                Debug.LogError("URDF file and ressources must be placed in Assets Folder:\n" + Application.dataPath);
                return;
            }

            GameObject robotGameObject = new GameObject(robot.name);
            robotGameObject.AddComponent<UrdfRobot>();

            UrdfAssetPathHandler.SetPackageRoot(Path.GetDirectoryName(robot.filename));
            UrdfMaterial.InitializeRobotMaterials(robot);
            UrdfPluginsExtensions.Create(robotGameObject.transform, robot.plugins);

            UrdfLinkExtensions.Create(robotGameObject.transform, robot.root);

            GameObjectUtility.SetParentAndAlign(robotGameObject, Selection.activeObject as GameObject);
            Undo.RegisterCreatedObjectUndo(robotGameObject, "Create " + robotGameObject.name);
            Selection.activeObject = robotGameObject;
        }

        #endregion

        #region Configure Robot

        public static void SetCollidersConvex(this UrdfRobot urdfRobot, bool convex)
        {
            foreach (MeshCollider meshCollider in urdfRobot.GetComponentsInChildren<MeshCollider>())
                meshCollider.convex = convex;
        }

        public static void SetRigidbodiesIsKinematic(this UrdfRobot urdfRobot, bool isKinematic)
        {
            foreach (Rigidbody rb in urdfRobot.GetComponentsInChildren<Rigidbody>())
                rb.isKinematic = isKinematic;
        }

        public static void SetUseUrdfInertiaData(this UrdfRobot urdfRobot, bool useUrdfData)
        {
            foreach (UrdfInertial urdfInertial in urdfRobot.GetComponentsInChildren<UrdfInertial>())
                urdfInertial.UseUrdfData = useUrdfData;
        }

        public static void SetRigidbodiesUseGravity(this UrdfRobot urdfRobot, bool useGravity)
        {
            foreach (Rigidbody rb in urdfRobot.GetComponentsInChildren<Rigidbody>())
                rb.useGravity = useGravity;
        }

        public static void GenerateUniqueJointNames(this UrdfRobot urdfRobot)
        {
            foreach (UrdfJoint urdfJoint in urdfRobot.GetComponentsInChildren<UrdfJoint>())
                urdfJoint.GenerateUniqueJointName();
        }

        #endregion

        #region Export

        public static void ExportRobotToUrdf(this UrdfRobot urdfRobot, string exportRootFolder, string exportDestination)
        {
            UrdfExportPathHandler.SetExportPath(exportRootFolder, exportDestination);

            urdfRobot.FilePath = Path.Combine(UrdfExportPathHandler.GetExportDestination(), urdfRobot.name + ".urdf");
    
            Robot robot = urdfRobot.ExportRobotData();
            if (robot == null) return;

            robot.WriteToUrdf();

            Debug.Log(robot.name + " was exported to " + UrdfExportPathHandler.GetExportDestination());

            UrdfMaterial.Materials.Clear();
            UrdfExportPathHandler.Clear();
            AssetDatabase.Refresh();
        }

        private static Robot ExportRobotData(this UrdfRobot urdfRobot)
        {
            Robot robot = new Robot(urdfRobot.FilePath, urdfRobot.gameObject.name);

            List<string> linkNames = new List<string>();

            foreach (UrdfLink urdfLink in urdfRobot.GetComponentsInChildren<UrdfLink>())
            {
                //Link export
                if (linkNames.Contains(urdfLink.name))
                {
                    EditorUtility.DisplayDialog("URDF Export Error",
                        "URDF export failed. There are several links with the name " +
                        urdfLink.name + ". Make sure all link names are unique before exporting this robot.",
                        "Ok");
                    return null;
                }
                robot.links.Add(urdfLink.ExportLinkData());
                linkNames.Add(urdfLink.name);

                //Joint export
                UrdfJoint urdfJoint = urdfLink.gameObject.GetComponent<UrdfJoint>();
                if (urdfJoint != null)
                    robot.joints.Add(urdfJoint.ExportJointData());
                else if (!urdfLink.IsBaseLink) 
                    //Make sure that links with no rigidbodies are still connected to the robot by a default joint
                    robot.joints.Add(UrdfJointExtensions.ExportDefaultJoint(urdfLink.transform));
            }

            robot.materials = UrdfMaterial.Materials.Values.ToList();
            robot.plugins = urdfRobot.GetComponentInChildren<UrdfPlugins>().ExportPluginsData();

            return robot;
        }

        #endregion
    }
}
