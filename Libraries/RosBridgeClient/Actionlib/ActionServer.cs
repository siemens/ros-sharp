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
    /*
     * This class is modelled after the ROS action server state machine
     * See details at
     * <http://wiki.ros.org/actionlib/DetailedDescription>
     */
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

        private ActionStatus actionStatus = ActionStatus.NO_GOAL;
        private string actionStatusText = "";

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

            UpdateAndPublishStatus(ActionStatus.NO_GOAL);
        }

        // Client Triggered Actions
        // When receive a new goal
        protected abstract void OnGoalReceived();
        private void GoalCallback(TActionGoal actionGoal)
        {
            action.action_goal = actionGoal;
            UpdateAndPublishStatus(ActionStatus.PENDING);
            OnGoalReceived();
        }

        // When the goal is cancelled by the client
        protected abstract void OnGoalRecalling(GoalID goalID);
        protected abstract void OnGoalPreempting();
        private void CancelCallback(GoalID goalID)
        {
            switch (actionStatus)
            {
                case ActionStatus.PENDING:
                    UpdateAndPublishStatus(ActionStatus.RECALLING);
                    OnGoalRecalling(goalID);
                    SetCanceled();
                    break;
                case ActionStatus.ACTIVE:
                    UpdateAndPublishStatus(ActionStatus.PREEMPTING);
                    OnGoalPreempting();
                    SetCanceled();
                    break;
                default:
                    LogWarning("Goal cannot be canceled under current state: " + actionStatus.ToString() + ". Ignored");
                    break;
            }
        }


        // Server Triggered Actions
        protected abstract void OnGoalActive();
        protected virtual void SetAccepted(string text = "")
        {
            switch (actionStatus)
            {
                case ActionStatus.PENDING:
                    UpdateAndPublishStatus(ActionStatus.ACTIVE, text);
                    OnGoalActive();
                    break;
                case ActionStatus.RECALLING:
                    UpdateAndPublishStatus(ActionStatus.PREEMPTING, text);
                    OnGoalPreempting();
                    break;
                default:
                    LogWarning("Goal cannot be set to be active under current state: " + actionStatus.ToString() + ". Ignored");
                    break;
            }
        }

        protected abstract void OnGoalRejected();
        protected virtual void SetRejected(string text = "")
        {
            switch (actionStatus)
            {
                case ActionStatus.PENDING:
                    UpdateAndPublishStatus(ActionStatus.REJECTED, text);
                    OnGoalRejected();
                    break;
                case ActionStatus.RECALLING:
                    UpdateAndPublishStatus(ActionStatus.REJECTED, text);
                    OnGoalRejected();
                    break;
                default:
                    LogWarning("Goal cannot be rejected under current state: " + actionStatus.ToString() + ". Ignored");
                    break;
            }
        }

        protected abstract void OnGoalSucceeded();
        protected virtual void SetSucceeded(TResult result = null, string text = "")
        {
            switch (actionStatus)
            {
                case ActionStatus.ACTIVE:
                    UpdateAndPublishStatus(ActionStatus.SUCCEEDED, text);
                    if (result != null)
                    {
                        action.action_result.result = result;
                    }
                    PublishResult();
                    OnGoalSucceeded();
                    break;
                case ActionStatus.PREEMPTING:
                    UpdateAndPublishStatus(ActionStatus.SUCCEEDED, text);
                    if (result != null)
                    {
                        action.action_result.result = result;
                    }
                    PublishResult();
                    OnGoalSucceeded();
                    break;
                default:
                    LogWarning("Goal cannot succeed under current state: " + actionStatus.ToString() + ". Ignored");
                    break;
            }
        }

        protected abstract void OnGoalAborted();
        protected virtual void SetAborted(TResult result = null, string text = "")
        {
            switch (actionStatus)
            {
                case ActionStatus.ACTIVE:
                    UpdateAndPublishStatus(ActionStatus.ABORTED, text);
                    OnGoalAborted();
                    break;
                case ActionStatus.PREEMPTING:
                    UpdateAndPublishStatus(ActionStatus.ABORTED, text);
                    OnGoalAborted();
                    break;
                default:
                    LogWarning("Goal cannot be aborted under current state: " + actionStatus.ToString() + ". Ignored");
                    break;
            }
            if (result != null)
            {
                action.action_result.result = result;
            }
        }

        protected abstract void OnGoalCanceled();
        protected void SetCanceled(TResult result = null, string text = "")
        {
            switch (actionStatus)
            {
                case ActionStatus.RECALLING:
                    UpdateAndPublishStatus(ActionStatus.RECALLED, text);
                    break;
                case ActionStatus.PREEMPTING:
                    UpdateAndPublishStatus(ActionStatus.PREEMPTED, text);
                    break;
                default:
                    LogWarning("Goal cannot be set to be canceled under current state: " + actionStatus.ToString() + ". Ignored");
                    break;
            }
            if (result != null) {
                action.action_result.result = result;
            }
            OnGoalCanceled();
        }


        protected void UpdateAndPublishStatus(ActionStatus actionStatus, string text = "")
        {
            this.actionStatus = actionStatus;
            this.actionStatusText = text;
            PublishStatus();
        }

        protected void PublishStatus()
        {
            if (actionStatus == ActionStatus.NO_GOAL)
            {
                socket.Publish(statusPublicationID, new GoalStatusArray());
            }
            else {
                socket.Publish(statusPublicationID,
                    new GoalStatusArray
                    {
                        status_list = new GoalStatus[]
                        {
                            new GoalStatus {
                                status = (byte)actionStatus,
                                text = actionStatusText
                            }
                        }
                    }
                );
            }
        }

        protected void PublishFeedback()
        {
            action.action_feedback.status.status = (byte)actionStatus;
            action.action_feedback.status.goal_id = action.action_goal.goal_id;
            socket.Publish(feedbackPublicationID, action.action_feedback);
        }

        protected void PublishResult()
        {
            action.action_feedback.status.status = (byte)actionStatus;
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

        protected ActionStatus GetStatus() {
            return actionStatus;
        }

        protected abstract void Log(string log);
        protected abstract void LogWarning(string log);

        public void Stop() {
            socket.Close(millisecondsTimestep);
        }
    }

    public class ActionServerException : Exception
    {
        public ActionServerException(string msg) : base(msg) { }
    }
}
