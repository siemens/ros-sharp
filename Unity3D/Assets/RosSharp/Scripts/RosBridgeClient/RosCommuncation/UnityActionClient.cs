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
using RosSharp.RosBridgeClient.MessageTypes.Actionlib;

namespace RosSharp.RosBridgeClient
{
    public abstract class UnityActionClient<TAction, TActionGoal, TActionResult, TActionFeedback, TGoal, TResult, TFeedback> : MonoBehaviour
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
        private string cancelPublicationID;
        private string goalPublicationID;
        private string statusSubscriptionID;
        private string feedbackSubscriptionID;
        private string resultSubscriptionID;

        protected GoalStatus goalStatus;
        protected TAction action;

        protected virtual void Start()
        {
            rosConnector = GetComponent<RosConnector>();
            
            cancelPublicationID    = rosConnector.RosSocket.Advertise<GoalID>(actionName + "/cancel");
            goalPublicationID      = rosConnector.RosSocket.Advertise<TActionGoal>(actionName + "/goal");
            statusSubscriptionID   = rosConnector.RosSocket.Subscribe<GoalStatusArray>(actionName + "/status",   StatusCallback,   (int)(timeStep * 1000));
            feedbackSubscriptionID = rosConnector.RosSocket.Subscribe<TActionFeedback>(actionName + "/feedback", FeedbackCallback, (int)(timeStep * 1000));
            resultSubscriptionID   = rosConnector.RosSocket.Subscribe<TActionResult>(actionName + "/result",     ResultCallback,   (int)(timeStep * 1000)); 
        }

        private void OnDestroy()
        {
            TerminateClient();
        }

        public void SendGoal()
        {
            action.action_goal = GetActionGoal();
            rosConnector.RosSocket.Publish(goalPublicationID, action.action_goal);
        }

        public void CancelGoal()
        {
            rosConnector.RosSocket.Publish(cancelPublicationID, action.action_goal.goal_id);
        }

        // Implement by user to attach GoalID
        protected abstract TActionGoal GetActionGoal();

        // Implement by user to handle status
        protected virtual void OnStatusUpdated() { }
        private void StatusCallback(GoalStatusArray actionGoalStatusArray)
        {
            if (actionGoalStatusArray.status_list.Length > 0)
            {
                goalStatus = actionGoalStatusArray.status_list[0];
            }

            OnStatusUpdated();
        }

        // Implement by user to handle feedback.
        protected abstract void OnFeedbackReceived();
        private void FeedbackCallback(TActionFeedback actionFeedback)
        {
            action.action_feedback = actionFeedback;
            OnFeedbackReceived();
        }

        // Implement by user to handle result.
        protected abstract void OnResultReceived();
        private void ResultCallback(TActionResult actionResult)
        {
            action.action_result = actionResult;
            OnResultReceived();
        }

        public void TerminateClient()
        {
            if(rosConnector != null)
                rosConnector.RosSocket.Close((int)(timeStep * 1000));
        }
    }
}
