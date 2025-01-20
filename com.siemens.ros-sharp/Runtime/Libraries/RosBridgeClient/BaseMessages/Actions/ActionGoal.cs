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

- Added ROS2 action support:
    - Removed GoalID goal_id property.
    - Added GoalInfo goalInfo property.
    - Renamed TGoal goal property to TGoal args.
    - Added string id property.
    - Added string action property.
    - Added string action_type property.
    - Added bool feedback property.
    - Added int fragment_size property.
    - Added string compression property.
    - Modified constructor to initialize goalInfo property.
    - Modified parameterized constructor to accept header and goalInfo parameters.

    © Siemens AG 2025, Mehmet Emre Cakal, emre.cakal@siemens.com/m.emrecakal@gmail.com
*/

using RosSharp.RosBridgeClient.MessageTypes.Std;

#if !ROS2
using RosSharp.RosBridgeClient.MessageTypes.Actionlib;
namespace RosSharp.RosBridgeClient
{
    public abstract class ActionGoal<TGoal> : Message where TGoal : Message
    {
        public Header header { get; set; }
        public GoalID goal_id { get; set; }
        public TGoal goal { get; set; }


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

#else
using RosSharp.RosBridgeClient.MessageTypes.Action;
namespace RosSharp.RosBridgeClient
{
    public abstract class ActionGoal<TGoal> : Message where TGoal : Message
    {
        public Header header { get; set; }
        public GoalInfo goalInfo { get; set; }
        public TGoal args { get; set; }
        public string id { get; set; }
        public string action { get; set; }
        public string action_type { get; set; }
        public bool feedback { get; set; }
        public int fragment_size { get; set; }
        public string compression { get; set; }



        public ActionGoal()
        {
            header = new Header();
            goalInfo = new GoalInfo();
        }

        public ActionGoal(Header header, GoalInfo goalInfo)
        {
            this.header = header;
            this.goalInfo = goalInfo;
        }
    }
}
#endif
