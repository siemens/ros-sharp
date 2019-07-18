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

using System.Threading;
using System.Collections.Generic;

using UnityEditor;
using UnityEngine;

using RosSharp.RosBridgeClient.Protocols;
using RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials;


namespace RosSharp.RosBridgeClient
{
    public class FibonacciActionServerComponent : MonoBehaviour
    {
        public string actionName = "fibonacci";
        public Protocol protocol = Protocol.WebSocketSharp;
        public string serverURL = "ws://192.168.137.195:9090";
        public RosSocket.SerializerEnum serializer = RosSocket.SerializerEnum.JSON;
        public int timeout = 10;
        public float timeStep = 0.2f;

        private FibonacciActionServer server;

        // Start is called before the first frame update
        private void Start()
        {
            server = new FibonacciActionServer(new FibonacciAction(), actionName, protocol, serverURL, serializer, timeout, timeStep);
        }

        // Update is called once per frame
        private void Update()
        {

        }

        private void OnDestroy()
        {
            server.Stop();
        }

        public string GetStatusString()
        {
            if (server == null)
            {
                return "";
            }
            return server.GetStatusString();
        }
    }

    public class FibonacciActionServer : ActionServer<FibonacciAction, FibonacciActionGoal, FibonacciActionResult, FibonacciActionFeedback, FibonacciGoal, FibonacciResult, FibonacciFeedback>
    {
        public FibonacciActionServer(FibonacciAction action, string actionName, Protocol protocol, string serverURL, RosSocket.SerializerEnum serializer, int timeout, float timeStep) : base(action, actionName, protocol, serverURL, serializer, timeout, timeStep) { }

        protected override bool IsGoalValid()
        {
            if (action.action_goal.goal.order <= 0)
            {
                Debug.LogError("Cannot generate fibonacci sequence of order less than 1");
            }
            return action.action_goal.goal.order > 0;
        }

        protected override void GoalHandler()
        {
            List<int> sequence = new List<int> { 0, 1 };

            action.action_feedback.feedback.sequence = sequence.ToArray();
            PublishFeedback();

            for (int i = 1; i < action.action_goal.goal.order; i++)
            {
                sequence.Add(sequence[i] + sequence[i - 1]);
                action.action_feedback.feedback.sequence = sequence.ToArray();
                PublishFeedback();

                Thread.Sleep((int)(timeStep * 1000));
            }

            UpdateAndPublishStatus(ActionStatus.SUCCEEDED);
            action.action_result.result.sequence = sequence.ToArray();
            PublishResult();
        }

        public string GetStatusString()
        {
            return actionStatus.ToString();
        }
    }

    [CustomEditor(typeof(FibonacciActionServerComponent))]
    public class FibonacciActionServerEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();

            EditorGUILayout.TextField("Status: ", ((FibonacciActionServerComponent)target).GetStatusString());

            Repaint();
        }
    }
}



