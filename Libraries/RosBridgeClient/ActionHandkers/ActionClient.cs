﻿/*
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
        protected int timeout;
        protected float timeStep;

        private readonly RosSocket socket;

        protected DateTime lastStatusUpdateTime = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);

        protected ManualResetEvent isResultReceived = new ManualResetEvent(false);

        private readonly string cancelPublicationID;
        private readonly string goalPublicationID;

        protected ActionStatus actionStatus;

        protected TAction action;

        public ActionClient(TAction action, string actionName, Protocol protocol, string serverURL, RosSocket.SerializerEnum serializer = RosSocket.SerializerEnum.JSON, int timeout = 10, float timeStep = 0.2f) {
            this.action = action;
            this.actionName = actionName;
            this.timeout = timeout;
            this.timeStep = timeStep;

            RosConnector connector = new RosConnector(serverURL, protocol, serializer, timeout);
            if (!connector.ConnectAndWait())
            {
                throw new Exception("Failed to connect to " + serverURL + " in " + timeout + " secs");
            }
            socket = connector.RosSocket;

            cancelPublicationID = socket.Advertise<GoalID>(actionName + "/cancel");
            goalPublicationID = socket.Advertise<TActionGoal>(actionName + "/goal");

            socket.Subscribe<GoalStatusArray>(actionName + "/status", StatusCallback, (int)(timeStep * 1000));
            socket.Subscribe<TActionFeedback>(actionName + "/feedback", FeedbackCallback, (int)(timeStep * 1000));
            socket.Subscribe<TActionResult>(actionName + "/result", ResultCallback, (int)(timeStep * 1000));
        }

        public bool WaitForServer(int timeout = -1) {
            DateTime waitStartTime = DateTime.Now;
            while (!IsServerUp()) { 
                Thread.Sleep((int)(timeStep * 1000));
                if (timeout > -1 && (DateTime.Now - waitStartTime).TotalSeconds > timeout) {
                    return false;
                }
            }
            return true;
        }

        public bool WaitForResult(int timeout = -1) {
            if (timeout < 0) {
                return isResultReceived.WaitOne();
            }
            return isResultReceived.WaitOne(timeout * 1000);
        }

        protected abstract void FeedbackHandler();       // Implement by user to handle feedback.

        protected abstract void ResultHandler();         // Implement by user to handle result.

        protected abstract bool IsServerUp(); // Implement by user to check if action server is up. Required for WaitForServer()

        public void SendGoal() {
            socket.Publish(goalPublicationID, action.action_goal);
            isResultReceived.Reset();
        }

        public void CancelGoal() {
            socket.Publish(cancelPublicationID, action.action_goal.goal_id);
        }

        protected void FeedbackCallback(TActionFeedback actionFeedback) {
            action.action_feedback = actionFeedback;
            actionStatus = (ActionStatus)actionFeedback.status.status;
            FeedbackHandler();
        }

        protected void ResultCallback(TActionResult actionResult) {
            action.action_result = actionResult;
            actionStatus = (ActionStatus)actionResult.status.status;
            ResultHandler();
            isResultReceived.Set();
        }

        protected void StatusCallback(GoalStatusArray actionGoalStatusArray) {
            if (actionGoalStatusArray.status_list.Length > 0) {
                actionStatus = (ActionStatus)actionGoalStatusArray.status_list[0].status;
            }
            lastStatusUpdateTime = DateTime.Now;
        }

        protected string FeedbackLogString() {
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
            socket.Close();
        }
    }
}
