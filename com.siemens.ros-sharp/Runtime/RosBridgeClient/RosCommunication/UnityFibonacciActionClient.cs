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

- Added ROS2 action support: ROS2 client registers goal with the proper message type.
- Added ReadOnlyAttribute and ReadOnlyDrawer for read-only fields in the Unity Editor: status, feedback, and result should not be modified by the user.
    © Siemens AG, 2025, Mehmet Emre Cakal,
*/

#if ROS2
using RosSharp.RosBridgeClient.MessageTypes.ActionTutorialsInterfaces;
#else
using RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials;
#endif

using UnityEngine;

namespace RosSharp.RosBridgeClient.Actionlib
{
    [RequireComponent(typeof(RosConnector))]
    public class UnityFibonacciActionClient : MonoBehaviour
    {
        private RosConnector rosConnector;
        public FibonacciActionClient fibonacciActionClient;

        public string actionName;
        public int fibonacciOrder = 20;

        [SerializeField, ReadOnly, Tooltip("Status (ReadOnly)")]
        public string status = "";
        [SerializeField, ReadOnly, Tooltip("Feedback (ReadOnly)")]
        public string feedback = "";
        [SerializeField, ReadOnly, Tooltip("Result (ReadOnly)")]
        public string result = "";

        private void Start()
        {
            rosConnector = GetComponent<RosConnector>();
            fibonacciActionClient = new FibonacciActionClient(actionName, rosConnector.RosSocket);
            fibonacciActionClient.Initialize();
        }

        private void Update()
        {
            status   = fibonacciActionClient.GetStatusString();
            feedback = fibonacciActionClient.GetFeedbackString();
            result   = fibonacciActionClient.GetResultString();
        }

        public void RegisterGoal()
        {
            #if !ROS2
            fibonacciActionClient.fibonacciOrder = fibonacciOrder;
            #else
            fibonacciActionClient.SetActionGoal(new FibonacciGoal(fibonacciOrder));
            #endif
        }
    }
}
