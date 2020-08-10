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

namespace RosBridgeClient.Messages
{

    public static class ActionMessageGenerator
    {

        public static void Generate(string ActionName, string RosPackageName,
            MessageElement[] GoalElements, MessageElement[] ResultElements, MessageElement[] FeedbackElements, string AssetPath)
        {
            SimpleMessageGenerator.Generate(ActionName + "Goal", RosPackageName, GoalElements, AssetPath);
            SimpleMessageGenerator.Generate(ActionName + "Feedback", RosPackageName, FeedbackElements, AssetPath);
            SimpleMessageGenerator.Generate(ActionName + "Result", RosPackageName, ResultElements, AssetPath);

            SimpleMessageGenerator.Generate(ActionName + "ActionGoal", RosPackageName,
                                    new MessageElement[] {    new MessageElement() { messageType = MessageType.Header,
                                                                                        messageName = "header",
                                                                                        isArray = false},

                                                                new MessageElement()  {messageType = MessageType.GoalID,
                                                                                        messageName = "goal_id",
                                                                                        isArray = false},

                                                                new MessageElement(    ActionName + "Goal",
                                                                                        "goal",
                                                                                        false)        }, AssetPath);

            SimpleMessageGenerator.Generate(ActionName + "ActionFeedback", RosPackageName,
                                    new MessageElement[] {    new MessageElement() { messageType = MessageType.Header,
                                                                                                    messageName = "header",
                                                                                                    isArray = false},

                                                                new MessageElement()  {messageType = MessageType.GoalStatus,
                                                                                        messageName = "status",
                                                                                        isArray = false},

                                                                new MessageElement(    ActionName + "Feedback",
                                                                                        "feedback",
                                                                                        false)        }, AssetPath);

            SimpleMessageGenerator.Generate(ActionName + "ActionResult", RosPackageName,
                                    new MessageElement[] {    new MessageElement() { messageType = MessageType.Header,
                                                                                                    messageName = "header",
                                                                                                    isArray = false},

                                                                new MessageElement()  {messageType = MessageType.GoalStatus,
                                                                                        messageName = "status",
                                                                                        isArray = false},

                                                                new MessageElement(    ActionName + "Result",
                                                                                        "result",
                                                                                        false)        }, AssetPath);
        }
    }
}
