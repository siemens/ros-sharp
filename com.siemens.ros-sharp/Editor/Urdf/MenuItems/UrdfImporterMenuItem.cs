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

#if UNITY_EDITOR

using System.IO;
using UnityEditor;
using UnityEngine;

namespace RosSharp.Urdf.Editor
{
    public class UrdfImporterMenuItem
    {
        [MenuItem("GameObject/3D Object/URDF Model (import)")]
        private static void CreateUrdfObject()
        {
            string urdfFile = EditorUtility.OpenFilePanel(
                "Import local URDF",
                Path.Combine(Path.GetDirectoryName(Application.dataPath),"Assets"),
                "urdf");

            if (urdfFile != "")
                UrdfRobotExtensions.Create(urdfFile);
        }
    }
}

#endif