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

using UnityEditor;
using UnityEngine;

namespace RosBridgeClient.Messages
{

    public class ActionMessageGenerator : MonoBehaviour
    {
        public string ActionName;
        public string RosPackageName;
        public MessageComponent[] GoalComponents;
        public MessageComponent[] ResultComponents;
        public MessageComponent[] FeedbackComponents;

        public void Generate()
        {
            SimpleMessageGenerator.Generate(ActionName + "Goal", RosPackageName, GoalComponents);
            SimpleMessageGenerator.Generate(ActionName + "Feedback", RosPackageName, FeedbackComponents);
            SimpleMessageGenerator.Generate(ActionName + "Result", RosPackageName, ResultComponents);

            SimpleMessageGenerator.Generate(ActionName + "ActionGoal", RosPackageName,
                                    new MessageComponent[] {    new MessageComponent() { messageType = MessageType.Header,
                                                                                        messageName = "header",
                                                                                        isArray = false},

                                                                new MessageComponent()  {messageType = MessageType.GoalID,
                                                                                        messageName = "goal_id",
                                                                                        isArray = false},

                                                                new MessageComponent(    ActionName + "Goal",
                                                                                        "goal",
                                                                                        false)        });


            SimpleMessageGenerator.Generate(ActionName + "ActionFeedback", RosPackageName,
                                    new MessageComponent[] {    new MessageComponent() { messageType = MessageType.Header,
                                                                                                    messageName = "header",
                                                                                                    isArray = false},

                                                                new MessageComponent()  {messageType = MessageType.GoalStatus,
                                                                                        messageName = "status",
                                                                                        isArray = false},

                                                                new MessageComponent(    ActionName + "Feedback",
                                                                                        "feedback",
                                                                                        false)        });

            SimpleMessageGenerator.Generate(ActionName + "ActionResult", RosPackageName,
                                    new MessageComponent[] {    new MessageComponent() { messageType = MessageType.Header,
                                                                                                    messageName = "header",
                                                                                                    isArray = false},

                                                                new MessageComponent()  {messageType = MessageType.GoalStatus,
                                                                                        messageName = "status",
                                                                                        isArray = false},

                                                                new MessageComponent(    ActionName + "Result",
                                                                                        "result",
                                                                                        false)        });

            AssetDatabase.Refresh();
        }
    }
}
