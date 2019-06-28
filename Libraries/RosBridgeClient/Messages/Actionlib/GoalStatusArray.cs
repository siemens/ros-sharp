/*
© Siemens AG, 2018-2019
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

using Newtonsoft.Json;

namespace RosSharp.RosBridgeClient.Messages.Actionlib
{
    public class GoalStatusArray : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "actionlib_msgs/GoalStatusArray";
        public Standard.Header header;
        public GoalStatus[] status_list;

        public GoalStatusArray()
        {
            header = new Standard.Header();
            status_list = new GoalStatus[] { new GoalStatus() };
        }
    }
}