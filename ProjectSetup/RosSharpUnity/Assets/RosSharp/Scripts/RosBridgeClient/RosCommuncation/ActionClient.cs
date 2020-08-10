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

using UnityEngine;

namespace RosSharp.RosBridgeClient
{

    public abstract class ActionClient<Tgoal, Tfeedback, Tresult> : MonoBehaviour
                                                                    where Tgoal : Message, new()
                                                                    where Tfeedback : Message, new()
                                                                    where Tresult : Message, new()
    {
        public string ActionName;
        public float TimeStep;

        private RosSocket rosSocket;
        private string CancelPublicationId;
        private string GoalPublicationId;
        private Messages.Actionlib.GoalID ActionGoalId;
        private Messages.Actionlib.GoalStatusArray ActionStatus;

        protected ActionServer<Tgoal, Tfeedback, Tresult>.ActionStates ActionState;
        protected Tgoal ActionGoal;
        protected Tfeedback ActionFeedback;
        protected Tresult ActionResult;

        public abstract Tgoal GetGoal(); // to be implemented by user

        protected virtual void Start()
        {
            rosSocket = GetComponent<RosConnector>().RosSocket;

            CancelPublicationId = rosSocket.Advertise<Messages.Actionlib.GoalID>(ActionName + "/cancel");
            GoalPublicationId = rosSocket.Advertise<Tgoal>(ActionName + "/goal");

            rosSocket.Subscribe<Messages.Actionlib.GoalStatusArray>(ActionName + "/status", StatusCallback, (int)(TimeStep * 1000));
            rosSocket.Subscribe<Tfeedback>(ActionName + "/feedback", FeedbackCallback, (int)(TimeStep * 1000));
            rosSocket.Subscribe<Tresult>(ActionName + "/result", ResultCallback, (int)(TimeStep * 1000));
        }

        public void SendGoal()
        {
            rosSocket.Publish(GoalPublicationId, GetGoal());
        }

        public void CancelGoal()
        {
            ActionGoalId = new Messages.Actionlib.GoalID();
            rosSocket.Publish(CancelPublicationId, ActionGoalId);
        }

        protected virtual void FeedbackCallback(Tfeedback feedback)
        {
            ActionFeedback = feedback;
        }

        protected virtual void ResultCallback(Tresult result)
        {
            ActionResult = result;
        }

        protected virtual void StatusCallback(Messages.Actionlib.GoalStatusArray actionStatus)
        {
            ActionStatus = actionStatus;
            ActionState = (ActionServer<Tgoal, Tfeedback, Tresult>.ActionStates)ActionStatus.status_list[0].status;
        }
    }
}