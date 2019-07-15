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

using RosSharp.RosBridgeClient.MessageTypes.Actionlib;

namespace RosSharp.RosBridgeClient
{
    public abstract class ActionClient<TAction, TActionGoal, TActionResult, TActionFeedback, TGoal, TResult, TFeedback>
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

        private RosSocket socket;

        private readonly string cancelPublicationID;
        private readonly string goalPublicationID;

        protected ActionStatus actionStatus;

        protected TAction action;

        public ActionClient(RosSocket socket, string actionName, TAction action) {
            this.socket = socket;
            this.actionName = actionName;
            this.action = action;

            cancelPublicationID = socket.Advertise<GoalID>(actionName + "/cancel");
            goalPublicationID = socket.Advertise<TActionGoal>(actionName + "/goal");

            socket.Subscribe<GoalStatusArray>(actionName + "/status", StatusCallback, (int)(timeStep * 1000));
            socket.Subscribe<TActionFeedback>(actionName + "/feedback", FeedbackCallback, (int)(timeStep * 1000));
            socket.Subscribe<TActionResult>(actionName + "/result", ResultCallback, (int)(timeStep * 1000));
        }

        public void SendGoal() {
            socket.Publish(goalPublicationID, action.action_goal);
        }

        public void CancelGoal() {
            socket.Publish(goalPublicationID, action.action_goal.goal_id);
        }

        protected void FeedbackCallback(TActionFeedback actionFeedback) {
            action.action_feedback = actionFeedback;
        }

        protected void ResultCallback(TActionResult actionResult) {
            action.action_result = actionResult;
        }

        protected void StatusCallback(GoalStatusArray actionGoalStatusArray) {
            actionStatus = (ActionStatus)actionGoalStatusArray.status_list[0].status;
        }
    }
}
