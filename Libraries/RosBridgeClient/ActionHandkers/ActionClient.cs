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

        protected bool isConnected = false;

        protected bool isServerUp = false;
        protected DateTime lastStatusUpdateTime = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);
        protected bool isUpdatingServerStatus = false;
        private readonly Thread updateServerStatus;

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
                return;
            }
            socket = connector.rosSocket;
            isConnected = true;

            cancelPublicationID = socket.Advertise<GoalID>(actionName + "/cancel");
            goalPublicationID = socket.Advertise<TActionGoal>(actionName + "/goal");

            socket.Subscribe<GoalStatusArray>(actionName + "/status", StatusCallback, (int)(timeStep * 1000));
            socket.Subscribe<TActionFeedback>(actionName + "/feedback", FeedbackCallback, (int)(timeStep * 1000));
            socket.Subscribe<TActionResult>(actionName + "/result", ResultCallback, (int)(timeStep * 1000));

            updateServerStatus = new Thread(UpdateServerStatus);
            StartUpdateServerStatus();
        }

        protected void WaitForServer() {
            while (!isServerUp) {
                Thread.Sleep((int)(timeStep * 1000));
            }
        }

        protected void WaitForResult() {
            while (actionStatus < ActionStatus.PREEMPTED) {
                Thread.Sleep((int)(timeStep * 1000));
            }
        }

        protected void SendGoal() {
            if (isConnected) {
                socket.Publish(goalPublicationID, action.action_goal);
            }
        }

        protected void CancelGoal() {
            if (isConnected) {
                socket.Publish(cancelPublicationID, action.action_goal.goal_id);
            }
        }

        protected void FeedbackCallback(TActionFeedback actionFeedback) {
            action.action_feedback = actionFeedback;
            actionStatus = (ActionStatus)actionFeedback.status.status;
        }

        protected void ResultCallback(TActionResult actionResult) {
            action.action_result = actionResult;
        }

        protected void StatusCallback(GoalStatusArray actionGoalStatusArray) {
            if (actionGoalStatusArray.status_list.Length > 0) {
                actionStatus = (ActionStatus)actionGoalStatusArray.status_list[0].status;
            }
            lastStatusUpdateTime = DateTime.Now;
            isServerUp = true;
        }

        protected void StartUpdateServerStatus()
        {
            isUpdatingServerStatus = true;
            updateServerStatus.Start();
        }

        protected void StopUpdateServerStatus()
        {
            isUpdatingServerStatus = false;
            updateServerStatus.Join();
        }

        protected void UpdateServerStatus() {
            while (isUpdatingServerStatus) {
                if ((long)(DateTime.Now - lastStatusUpdateTime).TotalMilliseconds > timeout * 1000)
                {
                    isServerUp = false;
                }
                Thread.Sleep((int)(timeStep * 1000));
            }
        }

        protected TResult GetResult() {
            return action.action_result.result;
        }
    }
}
