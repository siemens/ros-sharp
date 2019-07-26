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

using System;

using RosSharp.RosBridgeClient.Protocols;
using RosSharp.RosBridgeClient.MessageTypes.Actionlib;

namespace RosSharp.RosBridgeClient
{
    public abstract class ActionClient<TAction, TActionGoal, TActionResult, TActionFeedback, TGoal, TResult, TFeedback>
        where TAction : Action<TActionGoal, TActionResult, TActionFeedback, TGoal, TResult, TFeedback>
        where TActionGoal : ActionGoal<TGoal>
        where TActionResult : ActionResult<TResult>
        where TActionFeedback : ActionFeedback<TFeedback>
        where TGoal : Message
        where TResult : Message
        where TFeedback : Message
    {
        protected string actionName;
        protected float timeStep;

        private readonly RosSocket socket;

        private readonly string serverURL;

        protected DateTime lastStatusUpdateTime = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);

        private string cancelPublicationID;
        private string goalPublicationID;

        protected ActionStatus actionStatus;

        protected TAction action;

        public ActionClient(TAction action, string actionName, string serverURL, Protocol protocol = Protocol.WebSocketSharp, RosSocket.SerializerEnum serializer = RosSocket.SerializerEnum.JSON, float timeStep = 0.2f) {
            this.action = action;
            this.actionName = actionName;
            this.timeStep = timeStep;

            this.serverURL = serverURL;

            socket = new RosSocket(ProtocolInitializer.GetProtocol(protocol, serverURL), serializer);
        }

        public void Start() {
            cancelPublicationID = socket.Advertise<GoalID>(actionName + "/cancel");
            goalPublicationID = socket.Advertise<TActionGoal>(actionName + "/goal");

            socket.Subscribe<GoalStatusArray>(actionName + "/status", StatusCallback, (int)(timeStep * 1000));
            socket.Subscribe<TActionFeedback>(actionName + "/feedback", FeedbackCallback, (int)(timeStep * 1000));
            socket.Subscribe<TActionResult>(actionName + "/result", ResultCallback, (int)(timeStep * 1000));
        }

        public void SendGoal()
        {
            socket.Publish(goalPublicationID, action.action_goal);
        }

        public void CancelGoal()
        {
            socket.Publish(cancelPublicationID, action.action_goal.goal_id);
        }

        // Implement by user to wait for action server to be up
        protected abstract void WaitForActionServer();

        // Implement by user to wait for result
        protected abstract void WaitForResult();

        // Implement by user to handle feedback.
        protected abstract void FeedbackHandler();

        // Implement by user to handle result.
        protected abstract void ResultHandler();

        private void FeedbackCallback(TActionFeedback actionFeedback) {
            action.action_feedback = actionFeedback;
            actionStatus = (ActionStatus)actionFeedback.status.status;
            FeedbackHandler();
        }

        private void ResultCallback(TActionResult actionResult) {
            action.action_result = actionResult;
            actionStatus = (ActionStatus)actionResult.status.status;
            ResultHandler();
        }

        private void StatusCallback(GoalStatusArray actionGoalStatusArray) {
            if (actionGoalStatusArray.status_list.Length > 0) {
                actionStatus = (ActionStatus)actionGoalStatusArray.status_list[0].status;
            }
            lastStatusUpdateTime = DateTime.Now;
        }

        public string GetFeedbackLogString() {
            return
                "Feedback @ " + DateTime.Now + "\n" +
                action.action_feedback.ToString() + "\n" +
                "Server status: " + (ActionStatus)action.action_feedback.status.status + "\n" +
                "---\n";
        }

        public string GetResultLogString()
        {
            return
                "Result @ " + DateTime.Now + "\n" +
                action.action_result.ToString() + "\n" +
                "Server status: " + (ActionStatus)action.action_result.status.status + "\n" +
                "---\n";
        }

        public void Stop() {

            socket.Close();
        }
    }
}
