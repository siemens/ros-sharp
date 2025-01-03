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

//using System;
//using System.Linq;


//#if ROS2
//using RosSharp.RosBridgeClient.MessageTypes.ActionTutorialsInterfaces;
//using RosSharp.RosBridgeClient.MessageTypes.Action;
//#else
//using RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials;
//#endif

//namespace RosSharp.RosBridgeClient.Actionlib
//{
//    public class FibonacciActionClientNew : ActionClientNew<FibonacciAction, FibonacciActionGoal, FibonacciActionResult, FibonacciActionFeedback, FibonacciGoal, FibonacciResult, FibonacciFeedback>
//    {
//        public FibonacciActionClientNew(string actionName, string actionType, RosSocket rosSocket)
//        {
//            this.actionName = actionName;
//            this.actionType = actionType;
//            this.rosSocket = rosSocket;

//            action = new FibonacciAction();
//            goalStatus = new GoalStatus();
//        }

//        public override void SetActionGoal(FibonacciGoal fibonacciOrder, bool feedback = true, int fragmentSize = int.MaxValue, string compression = "none")
//        {
//            action.action_goal.action = actionName;
//            action.action_goal.action_type = actionType;
//            action.action_goal.args = fibonacciOrder;
//            action.action_goal.feedback = feedback;
//            action.action_goal.fragment_size = fragmentSize;
//            action.action_goal.compression = compression;
//        }

//        public override FibonacciActionGoal GetActionGoal()
//        {
//            return action.action_goal;
//        }

//        protected override void OnStatusUpdated()
//        {
//            // Not implemented for this particular application
//        }

//        protected override void OnFeedbackReceived()
//        {
//            Console.WriteLine("Feedback received: " + GetFeedbackString());
//        }

//        protected override void OnResultReceived()
//        {
//            Console.WriteLine("Result received: " + GetResultString());
//            Console.WriteLine("Status: " + GetStatusString());
//            Console.WriteLine("Result (success?): " + lastResultSuccess); // todo: needs better naming
//            Console.WriteLine("Frame ID: " + action.action_result.id);
//        }

//        public string GetStatusString()
//        {
//            if (goalStatus != null)
//            {
//                return ((ActionStatus)(goalStatus.status)).ToString();
//            }
//            return "";
//        }

//        public string GetFeedbackString()
//        {
//            if (action != null)
//                #if ROS2
//                return String.Join(", ", action.action_feedback.values.partial_sequence);
//                #else
//                return String.Join(", ", action.action_feedback.feedback.sequence);
//                #endif
//            return "";
//        }

//        public string GetResultString()
//        {
//            if (action != null)
//                return String.Join(", ", action.action_result.values.sequence);
//            return "";
//        }
//    }
//}
