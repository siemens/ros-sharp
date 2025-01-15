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

- Added ROS2 action support:
    - Namespace Updates: Replaced MessageTypes.Actionlib and MessageTypes.ActionlibTutorials with MessageTypes.ActionTutorialsInterfaces.
    - Goal Validation: Updated action.action_goal.goal.order to action.action_goal.args.order in the IsGoalValid method.
    - Feedback Handling: ROS2 uses action.action_feedback.values.partial_sequence instead of action.action_feedback.feedback.sequence.
    - Result Handling:
        - ROS2 uses action.action_result.values.sequence instead of action.action_result.result.sequence.
        - Added a new field action.action_result.result to indicate the success of the goal execution (true or false).
        - Updated result string generation in GetResultSequenceString.
    - Thread Execution:
        - Reduced Thread.Sleep in ExecuteFibonacciGoal from 1000ms (ROS1) to 500ms (ROS2).
        - Added a log message in ROS2 for feedback publication during goal execution.
    - Goal Reception:
        - ROS2 calls SetExecuting instead of SetAccepted in OnGoalReceived.
        - Added logging for accepted and rejected goals.
    - Goal Execution:
        - ROS2 implements OnGoalExecuting for starting goal execution, replacing OnGoalActive in ROS1.
        - Improved logging in OnGoalExecuting and during feedback/result handling.
    - Goal Cancellation:
        - Changed OnGoalPreempting (ROS1) to OnGoalCanceling (ROS2).
        - Enhanced logging for goal cancellation in ROS2.
    - Goal Status Updates:
        - Added ROS2 ActionStatus conventions.
        - Enhanced logging in OnGoalSucceeded, OnGoalAborted, and OnGoalCanceled.
    - General Logging:
         comprehensive log messages in various methods (e.g., ExecuteFibonacciGoal, OnGoalExecuting, OnGoalAborted) to improve debugging and monitoring.
    - Miscellaneous:
        - Removed redundant methods like OnGoalRecalling and simplified certain callbacks to align with ROS2's action handling.
        - Introduced consistent use of UpdateAndPublishStatus and PublishResult for publishing status and results.

    © Siemens AG 2025, Mehmet Emre Cakal, emre.cakal@siemens.com/m.emrecakal@gmail.com
*/

using System;
using System.Threading;
using System.Collections.Generic;


#if !ROS2
using RosSharp.RosBridgeClient.MessageTypes.Actionlib;
using RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials;


namespace RosSharp.RosBridgeClient.Actionlib
{
    public class FibonacciActionServer : ActionServer<FibonacciAction, FibonacciActionGoal, FibonacciActionResult, FibonacciActionFeedback, FibonacciGoal, FibonacciResult, FibonacciFeedback>
    {
        public string status = "";
        public string feedback = "";

        private ManualResetEvent isProcessingGoal = new ManualResetEvent(false);
        private Thread goalHandler;

        public FibonacciActionServer(string actionName, RosSocket rosSocket, Log log)
        {
            this.actionName = actionName;
            this.rosSocket = rosSocket;
            this.log = log;
            action = new FibonacciAction();
        }

        protected bool IsGoalValid()
        {
            return action.action_goal.goal.order >= 1;
        }

        private void ExecuteFibonacciGoal()
        {
            isProcessingGoal.Set();

            List<int> sequence = new List<int> { 0, 1 };

            action.action_feedback.feedback.sequence = sequence.ToArray();
            PublishFeedback();

            for (int i = 1; i < action.action_goal.goal.order; i++)
            {
                if (!isProcessingGoal.WaitOne(0))
                {
                    action.action_result.result.sequence = sequence.ToArray();
                    SetCanceled();
                    return;
                }

                sequence.Add(sequence[i] + sequence[i - 1]);

                action.action_feedback.feedback.sequence = sequence.ToArray();
                PublishFeedback();

                Thread.Sleep(1000);
            }

            action.action_result.result.sequence = sequence.ToArray();
            SetSucceeded();
        }

        public string GetFeedbackSequenceString()
        {
            if (action != null)
                return String.Join(",", action.action_feedback.feedback.sequence);

            return "";
        }

        protected override void OnGoalReceived()
        {
            if (IsGoalValid())
            {
                SetAccepted("Fibonacci Action Server: The goal has been accepted");
            }
            else
            {
                SetRejected("Fibonacci Action Server: Cannot generate fibonacci sequence of order less than 1");
            }
        }

        protected override void OnGoalRecalling(GoalID goalID)
        {
            // Left blank for this example
        }

        protected override void OnGoalRejected()
        {
            log("Cannot generate fibonacci sequence of order less than 1. Goal Rejected");
        }

        protected override void OnGoalActive()
        {
            goalHandler = new Thread(ExecuteFibonacciGoal);
            goalHandler.Start();
        }

        protected override void OnGoalPreempting()
        {
            isProcessingGoal.Reset();
            goalHandler.Join();
        }

        protected override void OnGoalSucceeded()
        {
            isProcessingGoal.Reset();
            Thread.Sleep((int)timeStep * 1000);
            UpdateAndPublishStatus(ActionStatus.SUCCEEDED);
        }

        protected override void OnGoalAborted()
        {
            // Left blank for this example
        }

        protected override void OnGoalCanceled()
        {
            PublishResult();
        }
    }
}
#else

using RosSharp.RosBridgeClient.MessageTypes.ActionTutorialsInterfaces;


namespace RosSharp.RosBridgeClient.Actionlib
{
    public class FibonacciActionServer : ActionServer<FibonacciAction, FibonacciActionGoal, FibonacciActionResult, FibonacciActionFeedback, FibonacciGoal, FibonacciResult, FibonacciFeedback>
    {
        public string status = "";
        public string feedback = "";

        private ManualResetEvent isProcessingGoal = new ManualResetEvent(false);
        private Thread goalHandler;

        public FibonacciActionServer(string actionName, RosSocket rosSocket, Log log)
        {
            this.actionName = actionName;
            this.rosSocket = rosSocket;
            this.log = log;
            action = new FibonacciAction();
        }

        protected bool IsGoalValid()
        {
            return action.action_goal.args.order >= 1;
        }

        private void ExecuteFibonacciGoal()
        {
            isProcessingGoal.Set();
            Thread.Sleep(500);
            List<int> sequence = new List<int> { 0, 1 };

            action.action_feedback.values.partial_sequence = sequence.ToArray();
            PublishFeedback();

            for (int i = 1; i < action.action_goal.args.order; i++)
            {
                if (!isProcessingGoal.WaitOne(0))
                {
                    action.action_result.values.sequence = sequence.ToArray();

                    if (this.GetStatus() != ActionStatus.STATUS_ABORTED)
                    {
                        SetCanceled();
                    }

                    return;
                }

                sequence.Add(sequence[i] + sequence[i - 1]);

                action.action_feedback.values.partial_sequence = sequence.ToArray();
                PublishFeedback();

                log("Fibonacci Action Server: Publishing feedback: " + GetFeedbackSequenceString());
                Thread.Sleep(500);
            }

            action.action_result.values.sequence = sequence.ToArray();
            action.action_result.result = true;
            SetSucceeded();

            log("Final result: " + GetResultSequenceString());
        }

        public string GetFeedbackSequenceString()
        {
            if (action != null)
                return String.Join(", ", action.action_feedback.values.partial_sequence);

            return "";
        }

        public string GetResultSequenceString()
        {
            if (action != null)
                return String.Join(", ", action.action_result.values.sequence);

            return "";
        }

        protected override void OnGoalReceived()
        {
            if (IsGoalValid())
            {
                SetExecuting(); 
            }
            else
            {
                SetAborted();
                log("Fibonacci Action Server: Cannot generate fibonacci sequence of order less than 1.");
            }
        }

        protected override void OnGoalExecuting()
        {
            log("Fibonacci Action Server: Accepted, executing.");
            goalHandler = new Thread(ExecuteFibonacciGoal);
            goalHandler.Start();
        }

        protected override void OnGoalCanceling()
        {
            log("Fibonacci Action Server: Cancelling.");
            isProcessingGoal.Reset();
            goalHandler.Join();
        }

        protected override void OnGoalSucceeded()
        {
            log("Fibonacci Action Server: Succeeded.");
            isProcessingGoal.Reset();
            UpdateAndPublishStatus(ActionStatus.STATUS_SUCCEEDED);
        }

        protected override void OnGoalAborted()
        {
            log("Fibonacci Action Server: Aborted.");
            isProcessingGoal.Reset();
            goalHandler.Join();
            action.action_result.result = false;
            PublishResult();
        }

        protected override void OnGoalCanceled()
        {
            log("Fibonacci Action Server: Canceled.");
            action.action_result.result = true;
            PublishResult();
        }
    }
}
#endif