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
        public string status;
        public string feedback;

        private void Start()
        {
            rosConnector = GetComponent<RosConnector>();
            fibonacciActionServer = new FibonacciActionServer(actionName, rosConnector.RosSocket, new MessageLogger(new MessageLogger.LogDelegate(x => Debug.Log(x))));
            fibonacciActionServer.Initialize();
        }

        private void Update()
        {
            fibonacciActionServer.PublishStatus();
            status = fibonacciActionServer.GetStatus().ToString();
            feedback = fibonacciActionServer.GetFeedbackSequenceString();
        }
    }

}