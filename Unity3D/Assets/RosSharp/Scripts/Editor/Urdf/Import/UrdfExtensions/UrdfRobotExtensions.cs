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

using UnityEngine;
using UnityEditor;

namespace RosSharp.Urdf.Import
{
    public static class UrdfRobotExtensions
    {
        public static GameObject Create(this Robot robot)
        {
            if (UrdfAssetPathHandler.IsValidAssetPath(robot.filename))
            { 
                Debug.LogError("URDF file and ressources must be placed in Assets Folder:\n" + Application.dataPath);
                return null;
            }

            UrdfAssetPathHandler.SetAssetRootFolder(robot);
            UrdfMaterialHandler.InitializeRobotMaterials(robot);

            GameObject gameObject = new GameObject(robot.name);
            robot.root.Create(gameObject);

            GameObjectUtility.SetParentAndAlign(gameObject, Selection.activeObject as GameObject);
            Undo.RegisterCreatedObjectUndo(gameObject, "Create " + gameObject.name);
            Selection.activeObject = gameObject;

            SetKinematic(gameObject, true);

            return gameObject;
        }

        public static void SetKinematic(GameObject robot, bool isKinematic)
        {
            Rigidbody[] rigidbodies = robot.GetComponentsInChildren<Rigidbody>();
            foreach (Rigidbody rigidbody in rigidbodies)
                rigidbody.isKinematic = isKinematic;
        }
    }
}
