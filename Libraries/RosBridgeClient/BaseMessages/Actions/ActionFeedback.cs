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
    - Removed GoalStatus status property.
    - Added TFeedback values property.
    - Added string id property.
    - Added string action property.
    - Modified constructor to initialize id and action properties.
    - Modified parameterized constructor to accept header, action, and id parameters.

    © Siemens AG 2025, Mehmet Emre Cakal, emre.cakal@siemens.com/m.emrecakal@gmail.com
*/

using RosSharp.RosBridgeClient.MessageTypes.Std;

#if !ROS2
using RosSharp.RosBridgeClient.MessageTypes.Actionlib;
namespace RosSharp.RosBridgeClient
{
    public abstract class ActionFeedback<TFeedback> : Message where TFeedback : Message
    {
        public Header header { get; set; }
        public GoalStatus status { get; set; }
        public TFeedback feedback { get; set; }

        public ActionFeedback()
        {
            header = new Header();
            status = new GoalStatus();
        }

        public ActionFeedback(Header header, GoalStatus status) {
            this.header = header;
            this.status = status;
        }
    }
}

#else
namespace RosSharp.RosBridgeClient
{
    public abstract class ActionFeedback<TFeedback> : Message where TFeedback : Message 
    {
        public Header header { get; set; }
        public TFeedback values { get; set; }
        public string id { get; set; }
        public string action { get; set; }

        public ActionFeedback()
        {
            header = new Header();
            id = "";
            action = "";
        }

        public ActionFeedback(Header header, string action, string id)
        {
            this.header = header;
            this.id = id;
            this.action = action;
        }
    }
}
#endif