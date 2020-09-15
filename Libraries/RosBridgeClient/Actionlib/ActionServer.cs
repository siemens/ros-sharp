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

using RosSharp.RosBridgeClient.MessageTypes.Actionlib;

namespace RosSharp.RosBridgeClient.Actionlib
{
    /*
     * ROS action server state machine is described in detail at;
     * <http://wiki.ros.org/actionlib/DetailedDescription>
     */

    public abstract class ActionServer<TAction, TActionGoal, TActionResult, TActionFeedback, TGoal, TResult, TFeedback>
        where TAction : Action<TActionGoal, TActionResult, TActionFeedback, TGoal, TResult, TFeedback>
        where TActionGoal : ActionGoal<TGoal>, new()
        where TActionResult : ActionResult<TResult>, new()
        where TActionFeedback : ActionFeedback<TFeedback>, new()
        where TGoal : Message, new()
        where TResult : Message, new()
        where TFeedback : Message, new()
    {
        public string actionName;
        public float timeStep;      // the rate(in s in between messages) at which to throttle the topics
        public RosSocket rosSocket;
        public Log log;

        private string statusPublicationID;
        private string feedbackPublicationID;
        private string resultPublicationID;
        private string cancelSubscriptionID;
        private string goalSubscriptionID;

        private ActionStatus actionStatus = ActionStatus.NO_GOAL;
        private string actionStatusText = "";
        protected TAction action;


        public void Initialize()
        {
            statusPublicationID = rosSocket.Advertise<GoalStatusArray>(actionName + "/status");
            feedbackPublicationID = rosSocket.Advertise<TActionFeedback>(actionName + "/feedback");
            resultPublicationID = rosSocket.Advertise<TActionResult>(actionName + "/result");
            cancelSubscriptionID = rosSocket.Subscribe<GoalID>(actionName + "/cancel", CancelCallback, (int)(timeStep * 1000));
            goalSubscriptionID = rosSocket.Subscribe<TActionGoal>(actionName + "/goal", GoalCallback, (int)(timeStep * 1000));

            UpdateAndPublishStatus(ActionStatus.NO_GOAL);
        }

        public void Terminate()
        {
            rosSocket.Unadvertise(statusPublicationID);
            rosSocket.Unadvertise(feedbackPublicationID);
            rosSocket.Unadvertise(resultPublicationID);
            rosSocket.Unsubscribe(cancelSubscriptionID);
            rosSocket.Unsubscribe(goalSubscriptionID);
        }

        public ActionStatus GetStatus()
        {
            return actionStatus;
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
                    break;
                case ActionStatus.ACTIVE:
                    UpdateAndPublishStatus(ActionStatus.PREEMPTING);
                    OnGoalPreempting();
                    break;
                default:
                    log("Goal cannot be canceled under current state: " + actionStatus.ToString() + ". Ignored");
                    break;
            }
        }

        // Server Triggered Actions
        protected abstract void OnGoalActive();
        protected void SetAccepted(string text = "")
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
                    log("Goal cannot be set to be active under current state: " + actionStatus.ToString() + ". Ignored");
                    break;
            }
        }

        protected virtual void OnGoalRejected() { }
        protected void SetRejected(string text = "")
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
                    log("Goal cannot be rejected under current state: " + actionStatus.ToString() + ". Ignored");
                    break;
            }
        }

        protected virtual void OnGoalSucceeded() { }
        protected void SetSucceeded(TResult result = null, string text = "")
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
                    log("Goal cannot succeed under current state: " + actionStatus.ToString() + ". Ignored");
                    break;
            }
        }

        protected virtual void OnGoalAborted() { }
        protected void SetAborted(string text = "")
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
                    log("Goal cannot be aborted under current state: " + actionStatus.ToString() + ". Ignored");
                    break;
            }
        }

        protected virtual void OnGoalCanceled() { }
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
                    log("Goal cannot be set to be canceled under current state: " + actionStatus.ToString() + ". Ignored");
                    return;
            }
            if (result != null)
            {
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

        public void PublishStatus()
        {
            if (actionStatus == ActionStatus.NO_GOAL)
            {
                rosSocket.Publish(statusPublicationID, new GoalStatusArray());
            }
            else
            {
                rosSocket.Publish(statusPublicationID,
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
            rosSocket.Publish(feedbackPublicationID, action.action_feedback);
        }

        protected void PublishResult()
        {
            action.action_feedback.status.status = (byte)actionStatus;
            action.action_result.status.goal_id = action.action_goal.goal_id;
            rosSocket.Publish(resultPublicationID, action.action_result);
        }
    }
}

