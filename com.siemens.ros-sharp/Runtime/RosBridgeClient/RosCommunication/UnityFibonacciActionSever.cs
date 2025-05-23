/*
© Siemens AG, 2019
Author: Berkay Alp Cakal (berkay_alp.cakal.ct@siemens.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

- Added ROS2 action support: ROS2 server does not need to publish status in the update loop.
- Added ReadOnlyAttribute and ReadOnlyDrawer for read-only fields in the Unity Editor: status and feedback should not be modified by the user.
    © Siemens AG, 2025, Mehmet Emre Cakal, emre.cakal@siemens.com/m.emrecakal@gmail.com
*/

using UnityEngine;

namespace RosSharp.RosBridgeClient.Actionlib
{
    [RequireComponent(typeof(RosConnector))]
    public class UnityFibonacciActionSever : MonoBehaviour
    {
        private RosConnector rosConnector;
        private FibonacciActionServer fibonacciActionServer;
        
        public string actionName;
        [SerializeField, ReadOnly, Tooltip("Status (ReadOnly)")]
        private string status;
        [SerializeField, ReadOnly, Tooltip("Feedback (ReadOnly)")]
        private string feedback;

        private void Start()
        {
            rosConnector = GetComponent<RosConnector>();
            fibonacciActionServer = new FibonacciActionServer(actionName, rosConnector.RosSocket, new Log(x => Debug.Log(x)));
            fibonacciActionServer.Initialize();
        }

        private void Update()
        {
            #if !ROS2
            fibonacciActionServer.PublishStatus();
            #endif

            status = fibonacciActionServer.GetStatus().ToString();
            feedback = fibonacciActionServer.GetFeedbackSequenceString();
        }
    }

    public class ReadOnlyAttribute : PropertyAttribute { }
}