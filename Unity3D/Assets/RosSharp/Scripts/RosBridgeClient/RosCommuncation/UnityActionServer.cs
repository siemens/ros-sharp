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

using System;
using UnityEngine;
using RosSharp.RosBridgeClient.Protocols;
using RosSharp.RosBridgeClient.MessageTypes.Actionlib;

namespace RosSharp.RosBridgeClient.Actionlib
{
    /*
     * For ROS action server state machine, please see details at
     * <http://wiki.ros.org/actionlib/DetailedDescription>
     */

    public abstract class UnityActionServer<TAction, TActionGoal, TActionResult, TActionFeedback, TGoal, TResult, TFeedback> : MonoBehaviour
        where TAction         : Action<TActionGoal, TActionResult, TActionFeedback, TGoal, TResult, TFeedback>
        where TActionGoal     : ActionGoal<TGoal>
        where TActionResult   : ActionResult<TResult>
        where TActionFeedback : ActionFeedback<TFeedback>
        where TGoal           : Message
        where TResult         : Message
        where TFeedback       : Message
    {
        public string actionName;
        public float timeStep;      // the rate(in s in between messages) at which to throttle the topics

        private RosConnector rosConnector;
        private string statusPublicationID;
        private string feedbackPublicationID;
        private string resultPublicationID;
        private string cancelSubscriptionID;
        private string goalSubscriptionID;

        private ActionStatus actionStatus = ActionStatus.NO_GOAL;
        private string actionStatusText = "";
        protected TAction action;

        protected virtual void Start()
        {
            rosConnector = GetComponent<RosConnector>();

            statusPublicationID   = rosConnector.RosSocket.Advertise<GoalStatusArray>(actionName + "/status");
            feedbackPublicationID = rosConnector.RosSocket.Advertise<TActionFeedback>(actionName + "/feedback");
            resultPublicationID   = rosConnector.RosSocket.Advertise<TActionResult>(actionName + "/result");
            cancelSubscriptionID  = rosConnector.RosSocket.Subscribe<GoalID>(actionName + "/cancel", CancelCallback, (int)(timeStep * 1000));
            goalSubscriptionID    = rosConnector.RosSocket.Subscribe<TActionGoal>(actionName + "/goal", GoalCallback, (int)(timeStep * 1000));

            UpdateAndPublishStatus(ActionStatus.NO_GOAL);
        }

        private void OnDestroy()
        {
            TerminateServer();
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
                    Debug.LogWarning("Goal cannot be canceled under current state: " + actionStatus.ToString() + ". Ignored");
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
                    Debug.LogWarning("Goal cannot be set to be active under current state: " + actionStatus.ToString() + ". Ignored");
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
                    Debug.LogWarning("Goal cannot be rejected under current state: " + actionStatus.ToString() + ". Ignored");
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
                    Debug.LogWarning("Goal cannot succeed under current state: " + actionStatus.ToString() + ". Ignored");
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
                    Debug.LogWarning("Goal cannot be aborted under current state: " + actionStatus.ToString() + ". Ignored");
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
                    Debug.LogWarning("Goal cannot be set to be canceled under current state: " + actionStatus.ToString() + ". Ignored");
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

        protected void PublishStatus()
        {
            if (actionStatus == ActionStatus.NO_GOAL)
            {
                rosConnector.RosSocket.Publish(statusPublicationID, new GoalStatusArray());
            }
            else
            {
                rosConnector.RosSocket.Publish(statusPublicationID,
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
            rosConnector.RosSocket.Publish(feedbackPublicationID, action.action_feedback);
        }

        protected void PublishResult()
        {
            action.action_feedback.status.status = (byte)actionStatus;
            action.action_result.status.goal_id = action.action_goal.goal_id;
            rosConnector.RosSocket.Publish(resultPublicationID, action.action_result);
        }

        protected ActionStatus GetStatus()
        {
            return actionStatus;
        }

        public void TerminateServer()
        {
            if(rosConnector != null)
                rosConnector.RosSocket.Close((int)(timeStep * 1000));
        }
    }
}
