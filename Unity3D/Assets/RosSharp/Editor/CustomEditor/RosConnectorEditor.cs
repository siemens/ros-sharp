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


namespace RosSharp.RosBridgeClient
{
    [CustomEditor(typeof(RosConnector))]
    public class RosConnectorEditor : Editor
    {
        private RosConnector rosConnector;

        private static string rosConnectorButtonOn = "Disconnect from ROS";
        private static string rosConnectorButtonOff = "Connect to ROS";
        private bool connected = false;

        private string rosConnectorButtonLabel = rosConnectorButtonOff;

        public override void OnInspectorGUI()
        {
            rosConnector = (RosConnector)target;
            DrawDefaultInspector();

            if (GUILayout.Button(rosConnectorButtonLabel))
            {
                if (connected)
                {
                    rosConnector.Connect(connected);
                    rosConnectorButtonLabel = rosConnectorButtonOff;
                    connected = false;
                }
                else
                {
                    rosConnector.Connect(connected);
                    rosConnectorButtonLabel = rosConnectorButtonOn;
                    connected = true;
                }

            }
            Application.runInBackground = true;
        }
    }
}