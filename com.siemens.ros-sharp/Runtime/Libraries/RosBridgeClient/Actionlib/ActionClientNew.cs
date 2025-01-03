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

//#if ROS2
//using RosSharp.RosBridgeClient.MessageTypes.Action;
//using System;

//namespace RosSharp.RosBridgeClient.Actionlib // This naming doesnt make sense but I dont want to break the backward compatibility
//{
//    public abstract class ActionClientNew<TAction, TActionGoal, TActionResult, TActionFeedback, TGoal, TResult, TFeedback>
//        where TAction : Action<TActionGoal, TActionResult, TActionFeedback, TGoal, TResult, TFeedback>
//        where TActionGoal : ActionGoal<TGoal>
//        where TActionResult : ActionResult<TResult> 
//        where TActionFeedback : ActionFeedback<TFeedback> 
//        where TGoal : Message
//        where TResult : Message
//        where TFeedback : Message
//    {
//        public RosSocket rosSocket;

//        public TAction action;
//        public string actionName;
//        public string actionType;

//        public GoalStatus goalStatus;
//        public bool lastResultSuccess;
//        public string frameId; 


//        public void Initialize()
//        {
//            // Not needed anymore
//        }

//        public void Terminate()
//        {
//            // Not needed anymore
//        }

//        public abstract void SetActionGoal(TGoal goal, bool feedback, int fragmentSize, string compression);
//        public abstract TActionGoal GetActionGoal();

//        public void SendGoal()
//        {
//            rosSocket.SendActionGoalRequest<TActionGoal, TGoal, TActionFeedback, TActionResult>(
//                action.action_goal,
//                ResultCallback,
//                FeedbackCallback);
//        }

//        public void CancelGoal(string frameId = null) 
//        {
//            rosSocket.CancelActionGoalRequest<TActionResult>(
//                frameId ?? this.frameId,
//                actionName,
//                ResultCallback);
//        }

        

//        // Implement by user to handle status
//        protected abstract void OnStatusUpdated();
//        private void StatusCallback(GoalStatusArray actionGoalStatusArray)
//        {
//            if (actionGoalStatusArray.status_list.Length > 0)
//            {
//                goalStatus = actionGoalStatusArray.status_list[actionGoalStatusArray.status_list.Length - 1];
//            }
//            OnStatusUpdated();
//        }

//        // Implement by user to handle feedback.
//        protected abstract void OnFeedbackReceived();
//        private void FeedbackCallback(TActionFeedback actionFeedbackValues)
//        {
//            action.action_feedback = actionFeedbackValues;
//            frameId = actionFeedbackValues.id;
//            OnFeedbackReceived();
//        }

//        // Implement by user to handle result.
//        protected abstract void OnResultReceived();
//        private void ResultCallback(TActionResult actionResult)
//        {
//            if (actionResult.result == false)
//            {
//                Console.WriteLine("Request failed!");
//            }
//            else
//            {
//                action.action_result = actionResult;
//                goalStatus.status = actionResult.status;
//                lastResultSuccess = actionResult.result;
//                frameId = actionResult.id;
//                OnResultReceived();
//            }

//        }
//    }
//}
//#endif