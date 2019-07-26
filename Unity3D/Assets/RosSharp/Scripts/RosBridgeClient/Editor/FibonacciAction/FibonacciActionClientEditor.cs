/*
© Siemens AG, 2019
Author: Sifan Ye (sifan.ye@siemens.com)

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

using UnityEditor;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
	[CustomEditor(typeof(FibonacciActionClientComponent))]
    public class FibonacciActionClientEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();

            if (GUILayout.Button("Send Goal"))
            {
                ((FibonacciActionClientComponent)target).SendGoal();
            }

            if (GUILayout.Button("Cancel Goal"))
            {
                ((FibonacciActionClientComponent)target).CancelGoal();
            }

            Repaint();
        }
    }
}