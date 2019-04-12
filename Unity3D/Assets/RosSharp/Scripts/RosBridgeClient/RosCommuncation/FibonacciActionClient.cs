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

using RosSharp.RosBridgeClient.Messages;

namespace RosSharp.RosBridgeClient
{

    public class FibonacciActionClient : ActionClient<FibonacciActionGoal,
                                                      FibonacciActionFeedback,
                                                      FibonacciActionResult>
    {
        public int Order;

        public override FibonacciActionGoal GetGoal()
        {
            return new FibonacciActionGoal() { goal = new FibonacciGoal { order = Order } };
        }

        public string PrintFeedback()
        {
            if (ActionFeedback == null)
                return "-";

            return PrintSequence(ActionFeedback.feedback.sequence);
        }

        public string PrintResult()
        {
            if (ActionResult == null)
                return "-";

            return PrintSequence(ActionResult.result.sequence);
        }

        public string PrintStatus()
        {
            return ActionState.ToString();
        }

        private static string PrintSequence(int[] intArray)
        {
            string result = "";
            for (int i = 0; i < intArray.Length; i++)
                result += " " + intArray[i];
            return result;
        }
    }

}