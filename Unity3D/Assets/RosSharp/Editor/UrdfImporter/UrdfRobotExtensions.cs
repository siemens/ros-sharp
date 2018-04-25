/*
© Siemens AG, 2017
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)

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

using System.IO;
using System.Xml;
using UnityEngine;
using UnityEditor;

namespace RosSharp.UrdfImporter
{
    public static class RobotCreator
    {
        public static GameObject Create(string filename)
        {
            Robot robot = new Robot(filename);
            return robot.Create();
        }
    }

    public static class UrdfRobotExtensions
    {
        public static GameObject Create(this Robot robot)
        {
            if (UrdfAssetDatabase.GetAssetPath(robot.filename) == null)
            {
                if (EditorUtility.DisplayDialog("Import URDF into Unity?",
                    "The selected URDF is not currently part of the Unity project. Import the URDF and assets into Unity?", "Import URDF", "Cancel"))
                {
                    if (!ImportUrdfToUnity(robot))
                    {
                        // Failed to import URDF into Unity.
                        return null;
                    }
                    AssetDatabase.Refresh();
                    robot = new Robot(robot.filename); // reinitialize robot
                }
                else
                {
                    EditorUtility.DisplayDialog("URDF Import Error", 
                        "URDF and resources must be placed here:\n" + Application.dataPath +
                        "\n\nPlease see the wiki:\nhttps://github.com/siemens/ros-sharp/wiki","OK") ;
                    return null;
                }
                
            }
            
            UrdfAssetDatabase.Initialize(robot);

            GameObject gameObject = new GameObject(robot.name);
            robot.root.Create(gameObject);

            GameObjectUtility.SetParentAndAlign(gameObject, Selection.activeObject as GameObject);
            Undo.RegisterCreatedObjectUndo(gameObject, "Create " + gameObject.name);
            Selection.activeObject = gameObject;

            setKinematic(gameObject, true);

            return gameObject;
        }

        public static void setKinematic(GameObject robot, bool isKinematic)
        {
            Rigidbody[] rigidbodies = robot.GetComponentsInChildren<Rigidbody>();
            foreach (Rigidbody rigidbody in rigidbodies)
                rigidbody.isKinematic = isKinematic;
        }

        public static bool ImportUrdfToUnity(Robot robot)
        {
            // Create URDF folder
            if (!AssetDatabase.IsValidFolder("Assets/Urdf"))
            {
                AssetDatabase.CreateFolder("Assets", "Urdf");
            }

            // Create folder for URDF
            if (!AssetDatabase.IsValidFolder("Assets/Urdf/" + robot.name))
            {
                AssetDatabase.CreateFolder("Assets/Urdf", robot.name);
            }
            // TODO: if folder exists, check if robot should be updated.

            // Create folder for meshes
            if (!AssetDatabase.IsValidFolder("Assets/Urdf/" + robot.name + "/meshes"))
            {
                AssetDatabase.CreateFolder("Assets/Urdf/" + robot.name, "meshes");
            }

            // Copy URDF into folder
            string unity_urdf_file_path = Path.Combine(Application.dataPath + "/Urdf/" + robot.name, robot.name + ".urdf");
            unity_urdf_file_path = unity_urdf_file_path.Replace(Path.AltDirectorySeparatorChar, Path.DirectorySeparatorChar);

            // Find the root of the description package
            string urdf_package_path = robot.filename;
            bool package_dir_found = false;
            do
            {
                // First loop, remove file name from string, next loops remove last directory from string
                urdf_package_path = urdf_package_path.Remove(urdf_package_path.LastIndexOf(Path.AltDirectorySeparatorChar));
                
                if (File.Exists(Path.Combine(urdf_package_path, "package.xml")))
                {
                    package_dir_found = true;
                    break;
                }
            } while (urdf_package_path.LastIndexOf(Path.AltDirectorySeparatorChar) != -1);

            if (!package_dir_found)
            {
                // Ask user to manually locate the package path
                if (EditorUtility.DisplayDialog("Import URDF Problem ",
                    "Could not find the root of the robot_description package. Select folder?", "Yes", "Cancel Import"))
                {
                    urdf_package_path = EditorUtility.OpenFolderPanel("Select URDF root folder", "", "");
                }
                else
                {
                    // Abort the import process if user cancels.
                    return false;
                }
            }
            string package_name = urdf_package_path.Remove(0, urdf_package_path.LastIndexOf(Path.AltDirectorySeparatorChar) + 1);

            // Load XML and copy mesh files
            XmlDocument urdf_xml = new XmlDocument();
            urdf_xml.Load(robot.filename);
            XmlNodeList mesh_nodes = urdf_xml.GetElementsByTagName("mesh");

            for (int i = 0; i < mesh_nodes.Count; i++)
            {
                string mesh_file_path = mesh_nodes[i].Attributes["filename"].Value;
                mesh_file_path = mesh_file_path.Replace("package://" + package_name, urdf_package_path);
                string mesh_file_name = mesh_file_path.Remove(0, mesh_file_path.LastIndexOf(Path.AltDirectorySeparatorChar) + 1);

                // Copy URDF into folder
                string unity_mesh_file_path = Application.dataPath + "/Urdf/" + robot.name + "/meshes/" + mesh_file_name;
                unity_mesh_file_path = unity_mesh_file_path.Replace(Path.AltDirectorySeparatorChar, Path.DirectorySeparatorChar);
                mesh_file_path = mesh_file_path.Replace(Path.AltDirectorySeparatorChar, Path.DirectorySeparatorChar);
                if (!File.Exists(unity_mesh_file_path))
                {
                    File.Copy(@mesh_file_path, @unity_mesh_file_path);
                }

                // Update URDF mesh path
                mesh_nodes[i].Attributes["filename"].Value = "package://" + robot.name + "/meshes/" + mesh_file_name;
            }
            // Save modified URDF to Assets folder
            urdf_xml.Save(Application.dataPath + "/Urdf/" + robot.name + "/" + robot.name + ".urdf");

            // Update robot.filename
            robot.filename = unity_urdf_file_path;

            return true;
        }
    }
}
