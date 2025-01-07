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
//using System.Threading;
//using System.Collections.Generic;


//using RosSharp.RosBridgeClient.MessageTypes.ActionTutorialsInterfaces;


//namespace RosSharp.RosBridgeClient.Actionlib
//{
//    public class FibonacciActionServerNew : ActionServerNew<FibonacciAction, FibonacciActionGoal, FibonacciActionResult, FibonacciActionFeedback, FibonacciGoal, FibonacciResult, FibonacciFeedback>
//    {
//        public string status = "";
//        public string feedback = "";

//        private ManualResetEvent isProcessingGoal = new ManualResetEvent(false);
//        private Thread goalHandler;

//        public FibonacciActionServerNew(string actionName, RosSocket rosSocket, Log log)
//        {
//            this.actionName = actionName;
//            this.rosSocket = rosSocket;
//            this.log = log;
//            action = new FibonacciAction();
//        }

//        protected bool IsGoalValid()
//        {
//            return action.action_goal.args.order >= 1;
//        }

//        private void ExecuteFibonacciGoal()
//        {
//            isProcessingGoal.Set();
//            Thread.Sleep(500);
//            List<int> sequence = new List<int> { 0, 1 };

//            #if ROS2
//            action.action_feedback.values.partial_sequence = sequence.ToArray();
//            #else
//            action.action_feedback.feedback.sequence = sequence.ToArray();
//            #endif

//            PublishFeedback();

//            for (int i = 1; i < action.action_goal.args.order; i++)
//            {
//                if (!isProcessingGoal.WaitOne(0))
//                {
//                    action.action_result.values.sequence = sequence.ToArray();

//                    if (this.GetStatus() != ActionStatus.STATUS_ABORTED)
//                    {
//                        SetCanceled();
//                    }

//                    return;
//                }

//                sequence.Add(sequence[i] + sequence[i - 1]);

//                #if ROS2
//                action.action_feedback.values.partial_sequence = sequence.ToArray();
//                #else
//                action.action_feedback.feedback.sequence = sequence.ToArray();
//                #endif

//                PublishFeedback();
//                log("Fibonacci Action Server: Publishing feedback: " + GetFeedbackSequenceString());
//                Thread.Sleep(500);
//            }

//            action.action_result.values.sequence = sequence.ToArray();
//            action.action_result.result = true;
//            SetSucceeded();
            
//            log("Final result: " + GetResultSequenceString());
//        }

//        public string GetFeedbackSequenceString()
//        {
//            if (action != null)
//                #if ROS2
//                return String.Join(", ", action.action_feedback.values.partial_sequence);
//                #else
//                return String.Join(", ", action.action_feedback.feedback.sequence);
//                #endif

//            return "";
//        }

//        public string GetResultSequenceString()
//        {
//            if (action != null)
//                return String.Join(", ", action.action_result.values.sequence);

//            return "";
//        }

//        protected override void OnGoalReceived()
//        {
//            //Console.WriteLine("TODO remove: Incoming goal: " + action.action_goal.args.order);
//            //Console.WriteLine("TODO remove: Incoming goal id: " + action.action_goal.id);
//            if (IsGoalValid()) 
//            {
//                SetExecuting(); // SetExectuing now
//            }
//            else
//            {
//                SetAborted(); 
//                log("Fibonacci Action Server: Cannot generate fibonacci sequence of order less than 1.");
//            }
//        }

//        protected override void OnGoalExecuting()
//        {
//            log("Fibonacci Action Server: Accepted, executing.");
//            goalHandler = new Thread(ExecuteFibonacciGoal);
//            goalHandler.Start();
//        }

//        protected override void OnGoalCancelling()
//        {
//            log("Fibonacci Action Server: Cancelling.");
//            isProcessingGoal.Reset();
//            goalHandler.Join();
//        }

//        protected override void OnGoalSucceeded()
//        {
//            log("Fibonacci Action Server: Succeeded.");
//            isProcessingGoal.Reset();
//            UpdateAndPublishStatus(ActionStatus.STATUS_SUCCEEDED);
//        }

//        protected override void OnGoalAborted()
//        {
//            log("Fibonacci Action Server: Aborted.");
//            isProcessingGoal.Reset();
//            goalHandler.Join();
//            action.action_result.result = false;
//            PublishResult();
//        }

//        protected override void OnGoalCanceled()
//        {
//            log("Fibonacci Action Server: Canceled.");
//            action.action_result.result = true;
//            PublishResult();
//        }
//    }
//}
