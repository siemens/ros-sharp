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
using System.Collections.Generic;

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
        private static string urdf_filename_;
        private static string robot_package_name_;

        private static string absolute_robot_package_path_;
        private static string unity_package_path_;
        private static string absolute_unity_package_path_;
        private static string absolute_unity_urdf_path_;

        public static GameObject Create(this Robot robot)
        {
            if (UrdfAssetDatabase.GetAssetPath(robot.filename) == null)
            {
                if (EditorUtility.DisplayDialog("Import URDF into Unity?",
                    "The selected URDF is not currently part of the Unity project. Import the URDF and assets into Unity?", "Import URDF", "Cancel"))
                {
                    if (!ImportUrdfToUnity(robot))
                    {
                        Debug.Log("URDF not imported into Unity.");
                        return null;
                    }
                    robot = new Robot(absolute_unity_urdf_path_); // reinitialize robot
                }
                else
                {
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

        // Import the URDF and mesh assets in to the Unity project
        public static bool ImportUrdfToUnity(Robot robot)
        {
            string dirSeparators = "\\/";
            urdf_filename_ = robot.filename.Substring(robot.filename.LastIndexOfAny(dirSeparators.ToCharArray()) + 1);

            if (!GetRobotPackageName(robot.filename)) return false;
            if (!GetRobotPackagePath(robot.filename)) return false;
            if (!GetUnityPackagePath()) return false;
            if (!ImportRobotPackage(robot.filename)) return false;

            return true;
        }

        // Get the robot URDF package path
        private static bool GetRobotPackagePath(string urdf_path)
        {
            if (urdf_path.Contains(robot_package_name_))
            {
                absolute_robot_package_path_ = urdf_path.Remove(urdf_path.IndexOf(robot_package_name_) + robot_package_name_.Length);
                return true;
            }
            else
            {
                EditorUtility.DisplayDialog("URDF Import Error",
                    "There was an error finding the robot description package path.", "OK");
                return false;
            }
        }

        // Gets the folder within the Unity project where the user would like to import the new robot description package 
        private static bool GetUnityPackagePath()
        {
            string absolute_robot_package_path = EditorUtility.OpenFolderPanel("Select location to import robot", "Assets", "");

            // Check that the location is inside of the Unity project's Asset folder
            if (absolute_robot_package_path.StartsWith(Application.dataPath))
            {
                // Check if the location is the Assets folder
                if (absolute_robot_package_path.Length > Application.dataPath.Length)
                {
                    unity_package_path_ = absolute_robot_package_path.Substring(Application.dataPath.Length + 1);
                    unity_package_path_ = Path.Combine(unity_package_path_, robot_package_name_);
                }
                unity_package_path_.Replace(Path.AltDirectorySeparatorChar, Path.DirectorySeparatorChar);
                absolute_unity_package_path_ = Path.Combine(Application.dataPath, unity_package_path_);
                absolute_unity_package_path_.Replace(Path.AltDirectorySeparatorChar, Path.DirectorySeparatorChar);
                return true;
            }
            else
            {
                EditorUtility.DisplayDialog("URDF Import Error",
                    "URDF and resources must be placed in the project's Assets folder\n" +
                    "\n\nPlease see the wiki:\nhttps://github.com/siemens/ros-sharp/wiki", "OK");
                return false;
            }
        }

        // Gets the name of the robot description package
        private static bool GetRobotPackageName(string robot_urdf)
        {
            // Load the urdf
            XmlDocument urdf_xml = new XmlDocument();
            urdf_xml.Load(robot_urdf);
            XmlNodeList mesh_nodes = urdf_xml.GetElementsByTagName("mesh");

            // Try to automatically find the package name
            if (mesh_nodes.Count > 0)
            {
                robot_package_name_ = mesh_nodes[0].Attributes["filename"].Value;
                robot_package_name_ = robot_package_name_.Replace("package://", string.Empty);
                int end_package_index = robot_package_name_.IndexOf("/");
                robot_package_name_ = robot_package_name_.Substring(0, end_package_index);
                return true;
            }
            else
            {
                // TODO: This limits importing URDFs to those with mesh files.
                EditorUtility.DisplayDialog("URDF Import Error", "Could not determine the URDF package name.", "OK");
                return false;
            }
        }

        // Imports the robot description package to into the Unity project
        private static bool ImportRobotPackage(string absolute_robot_urdf_path)
        {
            // Check if the URDF exists
            absolute_unity_urdf_path_ = Path.Combine(absolute_unity_package_path_, urdf_filename_);

            if (System.IO.File.Exists(absolute_unity_urdf_path_))
            {
                if (!EditorUtility.DisplayDialog("URDF file already exists!",
                    "A URDF with the same name already exists in this folder. Re-import URDF and mesh assets into Unity?", "Import URDF", "Cancel"))
                {
                    return false;
                }
            }
            System.IO.Directory.CreateDirectory(absolute_unity_package_path_);
            System.IO.File.Copy(absolute_robot_urdf_path, absolute_unity_urdf_path_, true);

            List<string> mesh_paths = GetListOfMeshFiles(absolute_robot_urdf_path);

            foreach (var mesh_path in mesh_paths)
            {
                string from = Path.Combine(absolute_robot_package_path_, mesh_path);
                string to = Path.Combine(absolute_unity_package_path_, mesh_path);
                System.IO.Directory.CreateDirectory(Path.GetDirectoryName(to));
                System.IO.File.Copy(from, to, true);
            }
            AssetDatabase.Refresh();
            return true;
        }

        // Gets a list of the relative mesh asset paths from the URDF
        private static List<string> GetListOfMeshFiles(string absolute_robot_urdf_path)
        {
            List<string> mesh_file_names = new List<string>();

            XmlDocument urdf_xml = new XmlDocument();
            urdf_xml.Load(absolute_robot_urdf_path);
            XmlNodeList mesh_nodes = urdf_xml.GetElementsByTagName("mesh");

            for (int i = 0; i < mesh_nodes.Count; i++)
            {
                string mesh_name = mesh_nodes[i].Attributes["filename"].Value;
                string replace_string = "package://" + robot_package_name_ + "/";
                mesh_name = mesh_name.Replace(replace_string, string.Empty);
                mesh_file_names.Add(mesh_name);
            }
            return mesh_file_names;
        }
    }
}
