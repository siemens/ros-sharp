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

namespace RosSharp.Urdf
{
    public class UrdfRobot : MonoBehaviour
    {
        private string filePath;

        public static void Create()
        {
            GameObject robotGameObject = new GameObject("Robot");
            robotGameObject.AddComponent<UrdfRobot>();

            UrdfLink urdfLink = UrdfLink.Create(robotGameObject.transform);
            urdfLink.name = "base_link";
            urdfLink.isBaseLink = true;
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

            UrdfLink.Create(robotGameObject.transform, robot.root);

            GameObjectUtility.SetParentAndAlign(robotGameObject, Selection.activeObject as GameObject);
            Undo.RegisterCreatedObjectUndo(robotGameObject, "Create " + robotGameObject.name);
            Selection.activeObject = robotGameObject;
        }

        #endregion

        #region Configure Robot

        public void SetCollidersConvex(bool convex)
        {
            foreach (MeshCollider meshCollider in GetComponentsInChildren<MeshCollider>())
                meshCollider.convex = convex;
        }

        public void SetRigidbodiesIsKinematic(bool isKinematic)
        {
            foreach (Rigidbody rb in GetComponentsInChildren<Rigidbody>())
                rb.isKinematic = isKinematic;
        }

        public void SetUseUrdfInertiaData(bool useUrdfData)
        {
            foreach (UrdfInertial urdfInertial in GetComponentsInChildren<UrdfInertial>())
                urdfInertial.UseUrdfData = useUrdfData;
        }

        public void SetRigidbodiesUseGravity(bool useGravity)
        {
            foreach (Rigidbody rb in GetComponentsInChildren<Rigidbody>())
                rb.useGravity = useGravity;
        }

        public void GenerateUniqueJointNames()
        {
            foreach (UrdfJoint urdfJoint in GetComponentsInChildren<UrdfJoint>())
                urdfJoint.GenerateUniqueJointName();
        }

        #endregion

        #region Export

        public void ExportRobotToUrdf(string exportRootFolder, string exportDestination)
        {
            UrdfExportPathHandler.SetExportPath(exportRootFolder, exportDestination);

            filePath = Path.Combine(UrdfExportPathHandler.GetExportDestination(), name + ".urdf");
    
            Robot robot = ExportRobotData();
            if (robot == null) return;

            robot.WriteToUrdf();

            Debug.Log(robot.name + " was exported to " + UrdfExportPathHandler.GetExportDestination());

            UrdfMaterial.Materials.Clear();
            UrdfExportPathHandler.Clear();
            AssetDatabase.Refresh();
        }

        private Robot ExportRobotData()
        {
            Robot robot = new Robot(filePath, gameObject.name);

            List<string> linkNames = new List<string>();

            foreach (UrdfLink urdfLink in gameObject.GetComponentsInChildren<UrdfLink>())
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
                else if (!urdfLink.isBaseLink) 
                    //Make sure that links with no rigidbodies are still connected to the robot by a default joint
                    robot.joints.Add(UrdfJoint.ExportDefaultJoint(urdfLink.transform));
            }

            robot.materials = UrdfMaterial.Materials.Values.ToList();

            return robot;
        }

        #endregion
    }
}
