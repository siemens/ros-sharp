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

namespace RosSharp
{
    [CustomEditor(typeof(KinematicPatcher))]
    public class KinematicPatcherEditor : Editor
    {
        private KinematicPatcher kinematicPatcher;

        private bool isKinematic = true;

        private static string kinematicButtonOn = "Set Rigidbodies Kinematic";
        private static string kinematicButtonOff = "Set Rigidbodies Non-Kinematic";

        private string kinematicButtonLabel = kinematicButtonOff;

        public override void OnInspectorGUI()
        {
            kinematicPatcher = (KinematicPatcher)target;
            DrawDefaultInspector();

            if (GUILayout.Button(kinematicButtonLabel))
            {
                if (isKinematic)
                {
                    isKinematic = false;
                    kinematicPatcher.SetKinematic(false);
                    kinematicButtonLabel = kinematicButtonOn;
                }
                else
                {
                    isKinematic = true;
                    kinematicPatcher.SetKinematic(true);
                    kinematicButtonLabel = kinematicButtonOff;
                }

            }
        }
    }
}
