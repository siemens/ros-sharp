﻿/*
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

using RosSharp.RosBridgeClient.MessageTypes.Std;
using RosSharp.RosBridgeClient.MessageTypes.Actionlib;

namespace RosSharp.RosBridgeClient
{
    public abstract class ActionGoal<TGoal> : Message where TGoal : Message
    {
        public Header header;
        public GoalID goal_id;
        public TGoal goal;

        public ActionGoal() {
            header = new Header();
            goal_id = new GoalID();
        }

        public ActionGoal(Header header, GoalID goal_id) {
            this.header = header;
            this.goal_id = goal_id;
        }
    }
}
