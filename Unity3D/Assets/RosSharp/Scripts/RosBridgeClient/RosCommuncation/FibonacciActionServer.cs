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

using System.Collections.Generic;
using System.Threading;
using UnityEngine;
using RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials;

namespace RosSharp.RosBridgeClient
{

    public class FibonacciActionServer : ActionServer<FibonacciActionGoal,
                                                      FibonacciActionFeedback,
                                                      FibonacciActionResult>
    {
        protected override void GoalHandle(FibonacciActionGoal actionGoal)
        {
            if (actionGoal.goal.order <= 0)
            {
                ActionState = ActionStates.Rejected;
                Debug.Log("The requested goal is invalid... ");
                return;
            }

            ActionState = ActionStates.Active;
            ActionResult = new FibonacciActionResult();
            ActionFeedback = new FibonacciActionFeedback();

            List<int> sequence = new List<int> { 0, 1 };
            ActionFeedback.feedback.sequence = sequence.ToArray();
            PublishFeedback();

            for (int i = 1; i <= actionGoal.goal.order; i++)
            {
                sequence.Add(sequence[i] + sequence[i - 1]);
                ActionFeedback.feedback.sequence = sequence.ToArray();
                PublishFeedback();
                Thread.Sleep(1000);
            }

            ActionState = ActionStates.Succeeded;

            ActionResult.result.sequence = ActionFeedback.feedback.sequence;
            PublishResult();
        }
    }

}