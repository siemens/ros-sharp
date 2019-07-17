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
using System.Threading;

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
        protected float timeStep;

        protected bool isRunning = false;

        protected ActionStatus actionStatus;

        private readonly RosSocket socket;

        private readonly string feedbackPublicationId;
        private readonly string statusPublicationId;
        private readonly string resultPublicationId;

        private Thread thread;

        protected TAction action;

        // User override for specific actions
        protected abstract void GoalHandler();
        protected abstract bool IsGoalValid();

        public ActionServer(TAction action, string actionName, Protocol protocol, string serverURL, RosSocket.SerializerEnum serializer = RosSocket.SerializerEnum.JSON, int timeout = 10, float timeStep = 0.1f) {
            this.action = action;
            this.actionName = actionName;

            this.timeStep = timeStep;

            RosConnector connector = new RosConnector(serverURL, protocol, serializer, timeout);
            if (!connector.ConnectAndWait()) {
                throw new Exception("Failed to connect to " + serverURL + " in " + timeout + " secs");
            }
            socket = connector.RosSocket;

            statusPublicationId = socket.Advertise<GoalStatusArray>(actionName + "/status");
            feedbackPublicationId = socket.Advertise<TActionFeedback>(actionName + "/feedback");
            resultPublicationId = socket.Advertise<TActionResult>(actionName + "/result");

            socket.Subscribe<GoalID>(actionName + "/cancel", CancelCallback, (int)(timeStep * 1000));
            socket.Subscribe<TActionGoal>(actionName + "/goal", GoalCallback, (int)(timeStep * 1000));

            UpdateAndPublishStatus(ActionStatus.PENDING);

            isRunning = true;
            new Thread(Spin).Start();
        }

        private void Spin() {
            while (isRunning) {
                PublishStatus();
                Thread.Sleep((int)(timeStep * 1000));
            }
        }

        // When receive a new goal
        private void GoalCallback(TActionGoal actionGoal)
        {
            if (actionStatus == ActionStatus.ACTIVE) {
                thread.Abort();
            }

            action.action_goal = actionGoal;
            action.action_result.status.goal_id = actionGoal.goal_id;
            action.action_feedback.status.goal_id = actionGoal.goal_id;

            if (IsGoalValid())
            {
                UpdateAndPublishStatus(ActionStatus.ACTIVE);
                thread = new Thread(() => GoalHandler());
                thread.Start();
            }
            else {
                UpdateAndPublishStatus(ActionStatus.REJECTED);
            }
        }

        private void CancelCallback(GoalID goalID)
        {
            if (actionStatus == ActionStatus.ACTIVE)
            {
                UpdateAndPublishStatus(ActionStatus.PREEMPTING);
                action.action_goal.goal_id = goalID;
                thread.Abort();
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
            socket.Publish(statusPublicationId,
                new GoalStatusArray
                {
                    status_list = new GoalStatus[]
                    {
                        new GoalStatus { status = (byte)actionStatus }
                    }
                }
            );
        }

        public void PublishFeedback()
        {
            action.action_feedback.status.status = (byte)actionStatus;
            socket.Publish(feedbackPublicationId, action.action_feedback);
        }

        public void PublishResult()
        {
            action.action_result.status.status = (byte)actionStatus;
            socket.Publish(resultPublicationId, action.action_result);
        }

        protected string FeedbackLogString()
        {
            return
                "Feedback @ " + DateTime.Now + "\n" +
                action.action_feedback.ToString() + "\n" +
                "Server status: " + (ActionStatus)action.action_feedback.status.status + "\n" +
                "---\n";
        }

        protected string ResultLogString()
        {
            return
                "Result @ " + DateTime.Now + "\n" +
                action.action_result.ToString() + "\n" +
                "Server status: " + (ActionStatus)action.action_result.status.status + "\n" +
                "---\n";
        }

        public void Stop() {
            isRunning = false;
            socket.Close();
        }
    }
}
