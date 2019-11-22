using System;
using System.Threading;
using System.Collections.Generic;

using RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials;
using RosSharp.RosBridgeClient.MessageTypes.Actionlib;

namespace RosSharp.RosBridgeClient.Actionlib
{
    public class FibonacciActionServer : ActionServer<FibonacciAction, FibonacciActionGoal, FibonacciActionResult, FibonacciActionFeedback, FibonacciGoal, FibonacciResult, FibonacciFeedback>
    {
        public string status = "";
        public string feedback = "";

        private ManualResetEvent isProcessingGoal = new ManualResetEvent(false);
        private Thread goalHandler;

        public FibonacciActionServer(string actionName, RosSocket rosSocket, MessageLogger logger)
        {
            this.actionName = actionName;
            this.rosSocket = rosSocket;
            this.messageLogger = logger;
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
            messageLogger.Log("Cannot generate fibonacci sequence of order less than 1. Goal Rejected");
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
