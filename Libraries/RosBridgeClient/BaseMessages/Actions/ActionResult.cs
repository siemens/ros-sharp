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

- Added ROS2 action support
    - Removed GoalStatus status property.
    - Added string action property.
    - Renamed TResult result property to TResult values.
    - Added sbyte status property.
    - Added GoalStatus goalStatus property.

    © Siemens AG 2025, Mehmet Emre Cakal, emre.cakal@siemens.com/m.emrecakal@gmail.com
*/

using RosSharp.RosBridgeClient.MessageTypes.Std;

#if !ROS2
using RosSharp.RosBridgeClient.MessageTypes.Actionlib;

namespace RosSharp.RosBridgeClient
{
    public abstract class ActionResult<TResult> : Message where TResult : Message
    {
        public Header header { get; set; }
        public GoalStatus status { get; set; }
        public TResult result { get; set; }

        public ActionResult() {
            header = new Header();
            status = new GoalStatus();
        }

        public ActionResult(Header header, GoalStatus status) {
            this.header = header;
            this.status = status;
        }
    }
}

#else
using RosSharp.RosBridgeClient.MessageTypes.Action;

namespace RosSharp.RosBridgeClient
{
    public abstract class ActionResult<TResult> : Message where TResult : Message
    {
        public Header header { get; set; }
        public string action { get; set; }
        public TResult values { get; set; }
        public sbyte status { get; set; }
        public GoalStatus goalStatus { get; set; }
        public bool result { get; set; }
        public string id { get; set; }


        public ActionResult()
        {
            header = new Header();
            status = 0;
            goalStatus = new GoalStatus();

        }

        public ActionResult(Header header, string action, sbyte status, bool result, string id)
        {
            this.header = header;
            this.status = status;
            this.action = action;
            this.values = values;
            this.goalStatus = new GoalStatus(new GoalInfo(), status);
            this.result = result;
            this.id = id;
        }
    }
}
#endif