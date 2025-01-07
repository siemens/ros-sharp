///*
//© Siemens AG, 2019
//Author: Berkay Alp Cakal (berkay_alp.cakal.ct@siemens.com)

//Licensed under the Apache License, Version 2.0 (the "License");
//you may not use this file except in compliance with the License.
//You may obtain a copy of the License at
//<http://www.apache.org/licenses/LICENSE-2.0>.
//Unless required by applicable law or agreed to in writing, software
//distributed under the License is distributed on an "AS IS" BASIS,
//WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//See the License for the specific language governing permissions and
//limitations under the License.
//*/

//using System.Threading;

//namespace RosSharp.RosBridgeClient.Actionlib
//{
//    /*
//     * ROS action server state machine is described in detail at;
//     * <http://wiki.ros.org/actionlib/DetailedDescription>
//     */

//    public abstract class ActionServerNew<TAction, TActionGoal, TActionResult, TActionFeedback, TGoal, TResult, TFeedback>
//        where TAction : Action<TActionGoal, TActionResult, TActionFeedback, TGoal, TResult, TFeedback>
//        where TActionGoal : ActionGoal<TGoal>
//        where TActionResult : ActionResult<TResult>
//        where TActionFeedback : ActionFeedback<TFeedback>
//        where TGoal : Message
//        where TResult : Message
//        where TFeedback : Message
//    {
//        public string actionName;
//        public RosSocket rosSocket;
//        public Log log;
//        protected TAction action;


//        private string ActionAdvertismentId;
//        private ActionStatus actionStatus = ActionStatus.STATUS_NO_GOAL;


//        public void Initialize()
//        {
//            ActionAdvertismentId = rosSocket.AdvertiseAction<TActionGoal, TActionFeedback, TActionResult>(actionName, GoalCallback, CancelCallback);

//            UpdateAndPublishStatus(ActionStatus.STATUS_NO_GOAL);
//        }

//        public void Terminate()
//        {
//            if (actionStatus == ActionStatus.STATUS_EXECUTING || actionStatus == ActionStatus.STATUS_CANCELING)
//            {
//                SetAborted();
//            }

//            Thread.Sleep(500);
//            rosSocket.UnadvertiseAction(ActionAdvertismentId);
//        }

//        public ActionStatus GetStatus()
//        {
//            return actionStatus;
//        }

//        // --- Client Triggered Actions ---
//        // When receive a new goal
//        protected abstract void OnGoalReceived();
//        private void GoalCallback(TActionGoal actionGoal)
//        {
//            action.action_goal = actionGoal;
//            UpdateAndPublishStatus(ActionStatus.STATUS_ACCEPTED); // PENDING
//            OnGoalReceived();
//        }

//        // When the goal is cancelled by the client
//        protected abstract void OnGoalCancelling(); //todo: do I need ID here?
//        private void CancelCallback(string frameId, string action)
//        {
//            switch (actionStatus)
//            {
//                case ActionStatus.STATUS_ACCEPTED: // todo: what to do here?

//                case ActionStatus.STATUS_EXECUTING:
//                    UpdateAndPublishStatus(ActionStatus.STATUS_CANCELING);
//                    OnGoalCancelling(); // OnGoalPreempting();
//                    break;
//                default:
//                    log("Goal cannot be 'cancelling' under current state: " + actionStatus.ToString() + ". Ignored");
//                    break;
//            }
//        }

//        // --- Server Triggered Actions ---
//        protected abstract void OnGoalExecuting(); //OnGoalActive
//        protected void SetExecuting()
//        {
//            switch (actionStatus)
//            {
//                case ActionStatus.STATUS_ACCEPTED: 
//                    UpdateAndPublishStatus(ActionStatus.STATUS_EXECUTING); 
//                    OnGoalExecuting();
//                    break;
//                default:
//                    log("Goal cannot be set to be 'executing' under current state: " + actionStatus.ToString() + ". Ignored");
//                    break;
//            }
//        }

//        protected virtual void OnGoalSucceeded() { }
//        protected void SetSucceeded(TResult result = null)
//        {
//            if (actionStatus == ActionStatus.STATUS_EXECUTING || actionStatus == ActionStatus.STATUS_CANCELING) //? WHY CANCEL
//            {
//                UpdateAndPublishStatus(ActionStatus.STATUS_SUCCEEDED);
//                if (result != null)
//                {
//                    action.action_result.values = result;
//                }
//                PublishResult();
//                OnGoalSucceeded();
//            }
//            else
//            {
//                log("Goal cannot 'succeed' under current state: " + actionStatus.ToString() + ". Ignored");
//            }
//        }

//        protected virtual void OnGoalAborted() { }
//        protected void SetAborted()
//        {
//            if (actionStatus == ActionStatus.STATUS_EXECUTING || actionStatus == ActionStatus.STATUS_CANCELING)
//            {
//                UpdateAndPublishStatus(ActionStatus.STATUS_ABORTED);
//                OnGoalAborted();
//            }
//            else
//            {
//                log("Goal cannot be 'aborted' under current state: " + actionStatus.ToString() + ". Ignored");
//            }
//        }

//        protected virtual void OnGoalCanceled() { }
//        protected void SetCanceled(TResult result = null)
//        {
//            if (actionStatus == ActionStatus.STATUS_CANCELING)
//            {
//                UpdateAndPublishStatus(ActionStatus.STATUS_CANCELED);
//                if (result != null)
//                {
//                    action.action_result.values = result;
//                }
//                OnGoalCanceled();
//            }
//            else
//            {
//                log("Goal cannot be 'canceled' under current state: " + actionStatus.ToString() + ". Ignored");
//            }
//        }

//        protected void UpdateAndPublishStatus(ActionStatus actionStatus) // not necessary anymore?
//        {
//            this.actionStatus = actionStatus;
//            //PublishStatus();
//        }

//        protected void PublishFeedback()
//        {
//            //action.action_feedback.status.status = (sbyte)actionStatus;
//            action.action_feedback.id = action.action_goal.id;
//            action.action_feedback.action = actionName;
//            //rosSocket.Publish(feedbackPublicationID, action.action_feedback);
//            rosSocket.RespondFeedback<TActionFeedback, TFeedback>(ActionAdvertismentId, action.action_feedback);
//        }

//        protected void PublishResult()
//        {
//            action.action_result.status = (sbyte)actionStatus;
//            action.action_result.goalStatus.status = (sbyte)actionStatus;
//            action.action_result.id = action.action_goal.id; 
//            action.action_result.action = actionName;
//            rosSocket.RespondResult<TActionResult, TResult>(ActionAdvertismentId, action.action_result); 
//        }
//    }
//}

