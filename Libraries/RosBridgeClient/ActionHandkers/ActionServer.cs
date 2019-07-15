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

using System.Threading;

using RosSharp.RosBridgeClient.MessageTypes.Actionlib;

namespace RosSharp.RosBridgeClient
{
    public abstract class ActionServer<TAction, TActionGoal, TActionResult, TActionFeedback, TGoal, TResult, TFeedback>
        where TAction : Action<TActionGoal, TActionResult, TActionFeedback, TGoal, TResult, TFeedback>
        where TActionGoal : ActionGoal<TGoal>
        where TActionResult : ActionResult<TResult>
        where TActionFeedback : ActionFeedback<TFeedback>
        where TGoal : Message
        where TResult : Message
        where TFeedback : Message
    {
        public string actionName;
        public float timeStep;

        public ActionStatus actionStatus;

        private readonly RosSocket socket;

        private readonly string feedbackPublicationId;
        private readonly string statusPublicationId;
        private readonly string resultPublicationId;

        private Thread thread;

        protected TAction action;

        protected abstract void GoalHandler(TActionGoal actionGoal);

        public ActionServer(RosSocket socket, TAction action, string actionName) {
            this.socket = socket;
            this.action = action;
            this.actionName = actionName;

            statusPublicationId = socket.Advertise<GoalStatusArray>(actionName + "/status");
            feedbackPublicationId = socket.Advertise<TActionFeedback>(actionName + "/feedback");
            resultPublicationId = socket.Advertise<TActionResult>(actionName + "/result");

            socket.Subscribe<GoalID>(actionName + "/cancel", CancelCallback, (int)(timeStep * 1000));
            socket.Subscribe<TActionGoal>(actionName + "/goal", GoalCallback, (int)(timeStep * 1000));
        }

        protected void GoalCallback(TActionGoal actionGoal)
        {
            if (actionStatus == ActionStatus.ACTIVE) {
                thread.Abort();
            }
            action.action_goal = actionGoal;
            thread = new Thread(() => GoalHandler(action.action_goal));
            thread.Start();
        }

        protected void CancelCallback(GoalID goalID)
        {
            if (actionStatus == ActionStatus.ACTIVE)
            {
                actionStatus = ActionStatus.PREEMPTING;
                action.action_goal.goal_id = goalID;
                thread.Abort();
                actionStatus = ActionStatus.PREEMPTED;
            }
        }

        protected void PublishStatus()
        {
            socket.Publish(statusPublicationId,
                new GoalStatusArray
                {
                    status_list = new GoalStatus[] 
                    {
                        new GoalStatus { status = (byte)actionStatus }
                    }
                }
            );
        }

        public void PublishFeedback()
        {
            socket.Publish(feedbackPublicationId, action.action_feedback);
        }

        public void PublishResult()
        {
            socket.Publish(resultPublicationId, action.action_result);
        }
    }
}
