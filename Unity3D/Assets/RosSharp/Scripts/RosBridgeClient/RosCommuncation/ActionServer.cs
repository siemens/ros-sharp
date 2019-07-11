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

using System.Threading;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{

    public abstract class ActionServer<Tgoal, Tfeedback, Tresult> : MonoBehaviour
                                                                    where Tgoal : Message
                                                                    where Tfeedback : Message
                                                                    where Tresult : Message
    {
        public string ActionName;
        public float TimeStep;

        public enum ActionStates { Pending, Active, Preempted, Succeeded, Aborted, Rejected, Preempting };
        public ActionStates ActionState;

        private RosSocket rosSocket;
        private string FeedbackPublicationId;
        private string StatusPublicationId;
        private string ResultPublicationId;
        private MessageTypes.Actionlib.GoalStatus ActionStatus;
        private MessageTypes.Actionlib.GoalID ActionGoalId;
        private Thread thread;

        protected Tgoal ActionGoal;
        protected Tfeedback ActionFeedback;
        protected Tresult ActionResult;

        protected abstract void GoalHandle(Tgoal goal); // to be implemented by user

        protected void Start()
        {
            rosSocket = GetComponent<RosConnector>().RosSocket;
            ActionState = ActionStates.Pending;

            rosSocket.Subscribe<MessageTypes.Actionlib.GoalID>(ActionName + "/cancel", CancelCallback, (int)(TimeStep * 1000));
            rosSocket.Subscribe<Tgoal>(ActionName + "/goal", GoalCallback, (int)(TimeStep * 1000));

            StatusPublicationId = rosSocket.Advertise<MessageTypes.Actionlib.GoalStatusArray>(ActionName + "/status");
            FeedbackPublicationId = rosSocket.Advertise<Tfeedback>(ActionName + "/feedback");
            ResultPublicationId = rosSocket.Advertise<Tresult>(ActionName + "/result");
        }

        protected virtual void Update()
        {
            PublishStatus();
        }

        protected void GoalCallback(Tgoal actionGoal)
        {
            if (ActionState == ActionStates.Active)
                thread.Abort(); // terminate existing goal handling process 

            ActionGoal = actionGoal;
            thread = new Thread(() => GoalHandle(ActionGoal));
            thread.Start();
        }

        protected void CancelCallback(MessageTypes.Actionlib.GoalID actionGoalId)
        {
            if (ActionState == ActionStates.Active)
            {
                ActionState = ActionStates.Preempting;
                ActionGoalId = actionGoalId;
                thread.Abort();
                ActionState = ActionStates.Preempted;
            }
        }

        protected void PublishStatus()
        {
            ActionStatus = new MessageTypes.Actionlib.GoalStatus() { status = (byte)ActionState };

            rosSocket.Publish(StatusPublicationId,
                new MessageTypes.Actionlib.GoalStatusArray { status_list = new MessageTypes.Actionlib.GoalStatus[] { ActionStatus } });
        }

        public void PublishFeedback()
        {
            rosSocket.Publish(FeedbackPublicationId, ActionFeedback);
        }

        public void PublishResult()
        {
            rosSocket.Publish(ResultPublicationId, ActionResult);
        }
    }
}