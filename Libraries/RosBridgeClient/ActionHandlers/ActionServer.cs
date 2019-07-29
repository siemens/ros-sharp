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
    public abstract class ActionServer<TAction, TActionGoal, TActionResult, TActionFeedback, TGoal, TResult, TFeedback>
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

        protected ActionStatus actionStatus;

        private readonly RosSocket socket;

        private readonly string serverURL;

        private string statusPublicationID;
        private string feedbackPublicationID;
        private string resultPublicationID;

        private string cancelSubscriptionID;
        private string goalSubscriptionID;

        protected TAction action;

        public ActionServer(TAction action, string actionName, Protocol protocol, string serverURL, RosSocket.SerializerEnum serializer = RosSocket.SerializerEnum.JSON, float secondsTimeout = 5f, float secondsTimestep = 0.1f) {
            this.action = action;
            this.actionName = actionName;

            this.millisecondsTimeout = (int)(secondsTimeout * 1000);
            this.millisecondsTimestep = (int)(secondsTimestep * 1000);

            this.serverURL = serverURL;

            socket = new RosSocket(ProtocolInitializer.GetProtocol(protocol, serverURL), serializer);
        }

        public void Start() {
            statusPublicationID = socket.Advertise<GoalStatusArray>(actionName + "/status");
            feedbackPublicationID = socket.Advertise<TActionFeedback>(actionName + "/feedback");
            resultPublicationID = socket.Advertise<TActionResult>(actionName + "/result");

            cancelSubscriptionID = socket.Subscribe<GoalID>(actionName + "/cancel", CancelCallback, millisecondsTimestep);
            goalSubscriptionID = socket.Subscribe<TActionGoal>(actionName + "/goal", GoalCallback, millisecondsTimestep);

            UpdateAndPublishStatus(ActionStatus.PENDING);
        }

        // Implemented by user to wait for goal
        protected abstract void WaitForGoal();

        // Implemented by user to check if a specific goal is valid
        protected abstract bool IsGoalValid();

        // Implemented by user to respond to a goal
        protected abstract void GoalHandler();

        // Implemented by user to respond to a goal cancellation request
        protected abstract void CancellationHandler();

        // When receive a new goal
        private void GoalCallback(TActionGoal actionGoal)
        {
            action.action_goal = actionGoal;
            UpdateAndPublishStatus(ActionStatus.ACTIVE);
            if (IsGoalValid())
            {
                GoalHandler();
            }
            else
            {
                UpdateAndPublishStatus(ActionStatus.REJECTED);
            }
        }

        // When the goal is cancelled by the client
        private void CancelCallback(GoalID goalID)
        {
            if (actionStatus == ActionStatus.ACTIVE)
            {
                UpdateAndPublishStatus(ActionStatus.PREEMPTING);
                action.action_goal.goal_id = goalID;
                CancellationHandler();
                UpdateAndPublishStatus(ActionStatus.PREEMPTED);
            }
        }

        protected void UpdateAndPublishStatus(ActionStatus actionStatus)
        {
            this.actionStatus = actionStatus;
            PublishStatus();
        }

        protected void PublishStatus()
        {
            socket.Publish(statusPublicationID,
                new GoalStatusArray
                {
                    status_list = new GoalStatus[]
                    {
                        new GoalStatus { status = (byte)actionStatus }
                    }
                }
            );
        }

        protected void PublishFeedback()
        {
            action.action_feedback.status.status = (byte)actionStatus;
            action.action_feedback.status.goal_id = action.action_goal.goal_id;
            socket.Publish(feedbackPublicationID, action.action_feedback);
        }

        protected void PublishResult()
        {
            action.action_result.status.status = (byte)actionStatus;
            action.action_result.status.goal_id = action.action_goal.goal_id;
            socket.Publish(resultPublicationID, action.action_result);
        }

        protected string GetFeedbackLogString()
        {
            return
                "Feedback @ " + DateTime.Now + "\n" +
                action.action_feedback.ToString() + "\n" +
                "Server status: " + (ActionStatus)action.action_feedback.status.status + "\n" +
                "---\n";
        }

        protected string GetResultLogString()
        {
            return
                "Result @ " + DateTime.Now + "\n" +
                action.action_result.ToString() + "\n" +
                "Server status: " + (ActionStatus)action.action_result.status.status + "\n" +
                "---\n";
        }

        public void Stop() {
            socket.Close(millisecondsTimestep);
        }
    }
}
