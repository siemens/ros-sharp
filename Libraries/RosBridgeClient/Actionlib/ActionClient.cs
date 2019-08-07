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

namespace RosSharp.RosBridgeClient.Actionlib
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

        protected int millisecondsTimeout;
        protected int millisecondsTimestep;

        private readonly RosSocket socket;

        private readonly string serverURL;

        protected DateTime lastStatusUpdateTime = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);

        private string cancelPublicationID;
        private string goalPublicationID;

        private string statusSubscriptionID;
        private string feedbackSubscriptionID;
        private string resultSubscriptionID;

        protected GoalStatus goalStatus;

        protected TAction action;

        public ActionClient(TAction action, string actionName, string serverURL, Protocol protocol = Protocol.WebSocketSharp, RosSocket.SerializerEnum serializer = RosSocket.SerializerEnum.JSON, float secondsTimeout = 5f,  float secondsTimestep = 0.2f) {
            this.action = action;
            this.actionName = actionName;

            this.millisecondsTimeout = (int)(secondsTimeout * 1000);
            this.millisecondsTimestep = (int)(secondsTimestep * 1000);

            this.serverURL = serverURL;

            socket = new RosSocket(ProtocolInitializer.GetProtocol(protocol, serverURL), serializer);
        }

        public void Start() {
            cancelPublicationID = socket.Advertise<GoalID>(actionName + "/cancel");
            goalPublicationID = socket.Advertise<TActionGoal>(actionName + "/goal");

            statusSubscriptionID = socket.Subscribe<GoalStatusArray>(actionName + "/status", StatusCallback, millisecondsTimestep);
            feedbackSubscriptionID = socket.Subscribe<TActionFeedback>(actionName + "/feedback", FeedbackCallback, millisecondsTimestep);
            resultSubscriptionID = socket.Subscribe<TActionResult>(actionName + "/result", ResultCallback, millisecondsTimestep);
        }

        public void SendGoal()
        {
            action.action_goal.goal_id.id = GoalID();
            socket.Publish(goalPublicationID, action.action_goal);
        }

        public void CancelGoal()
        {
            socket.Publish(cancelPublicationID, action.action_goal.goal_id);
        }

        // Implement by user to attach GoalID
        protected abstract string GoalID();

        // Implement by user to handle status
        protected virtual void OnStatusUpdated() { }
        private void StatusCallback(GoalStatusArray actionGoalStatusArray)
        {
            if (actionGoalStatusArray.status_list.Length > 0)
            {
                goalStatus = actionGoalStatusArray.status_list[0];
            }
            lastStatusUpdateTime = DateTime.Now;
            OnStatusUpdated();
        }

        // Implement by user to handle feedback.
        protected abstract void OnFeedbackReceived();
        private void FeedbackCallback(TActionFeedback actionFeedback) {
            action.action_feedback = actionFeedback;
            OnFeedbackReceived();
        }

        // Implement by user to handle result.
        protected abstract void OnResultReceived();
        private void ResultCallback(TActionResult actionResult) {
            action.action_result = actionResult;
            OnResultReceived();
        }

        protected virtual void Log(string log) { }
        protected virtual void LogWarning(string log) { }
        protected virtual void LogError(string log) { }

        public void Stop() {
            socket.Close(millisecondsTimestep);
        }
    }
}
