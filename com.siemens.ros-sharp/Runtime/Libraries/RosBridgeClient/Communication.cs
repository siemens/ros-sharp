/*
© Siemens AG, 2017-2019
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)

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
    - Added `ActionAdvertisement` for advertising actions (Provider side): 
        - Used to notify clients about the availability of a specific action with its type.  
    - Added `ActionUnadvertisement` for unadvertising actions (Provider side): 
        - Used to stop the advertisement of an action, removing it from the available actions list.  
    - Added `SendActionGoal<T>` for sending action goals (Consumer side): 
        - Allows clients to send a goal for a specific action, with optional feedback and compression settings.  
    - Added `CancelActionGoal` for canceling action goals (Consumer side): 
        - Enables clients to cancel a previously sent goal for an action, identified by its ID.  
    - Added `ActionFeedbackResponse<T>` for sending feedback on actions (Provider side): 
        - Used by the server to send periodic feedback about the progress of a goal to the client.  
    - Added `ActionResultResponse<T>` for sending action result responses (Provider side): 
        - Communicates the final result, status, and success or failure of an action goal to the client.  

    © Siemens AG 2025, Mehmet Emre Cakal, emre.cakal@siemens.com/m.emrecakal@gmail.com
*/


using System.Text.Json.Serialization;

namespace RosSharp.RosBridgeClient
{
    internal abstract class Communication
    {
        public string op { get; set; } // required
        public string id { get; set; } // optional

        internal Communication(string id = null)
        {
            this.id = id;
        }
    }

    internal class Advertisement : Communication
    {
        public string topic { get; set; } // required
        public string type  { get; set; } // required

        internal Advertisement(string id, string topic, string type) : base(id)
        {
            this.op = "advertise";
            this.topic = topic;
            this.type = type;
        }
    }

    internal class Unadvertisement : Communication
    {
        public string topic { get; set; } // required

        internal Unadvertisement(string id, string topic) : base(id)
        {
            this.op = "unadvertise";
            this.topic = topic;
        }
    }

    internal class Publication<T> : Communication where T : Message
    {
        public string topic { get; set; } // required
        public T msg { get; set; } // required

        internal Publication(string id, string topic, T msg) : base(id)
        {
            this.op = "publish";
            this.topic = topic;
            this.msg = msg;
        }
    }

    internal class Subscription : Communication
    {
        public string topic { get; set; } // required
        public string type { get; set; } // optional
        public int throttle_rate { get; set; } // optional
        public int queue_length { get; set; } // optional
        public int fragment_size { get; set; } // optional
        public string compression { get; set; } // optional

        internal Subscription(string id, string topic, string type, int throttle_rate = 0, int queue_length = 1, int fragment_size = int.MaxValue, string compression = "none") : base(id)
        {
            this.op = "subscribe";
            this.topic = topic;
            this.type = type;
            this.throttle_rate = throttle_rate;
            this.queue_length = queue_length;
            this.fragment_size = fragment_size;
            this.compression = compression;
        }
    }

    internal class Unsubscription : Communication
    {
        public string topic { get; set; } // required

        internal Unsubscription(string id, string topic) : base(id)
        {
            this.op = "unsubscribe";
            this.topic = topic;
        }
    }

    internal class ServiceCall<T> : Communication where T : Message
    { 
        public string service { get; set; } // required
        public T args { get; set; } // optional
        public int fragment_size { get; set; } // optional
        public string compression { get; set; } // optional

        public ServiceCall(string id, string service, T args, int fragment_size = int.MaxValue, string compression = "none") : base(id)
        {
            this.op = "call_service";
            this.service = service;
            this.args = args;
            this.fragment_size = fragment_size;
            this.compression = compression;
        }
    }

    internal class ServiceResponse<T> : Communication where T : Message
    {
        public string service { get; set; } // required
        public T values { get; set; } // optional
        public bool result { get; set; } // required

        internal ServiceResponse(string id, string service, T values, bool Result) : base(id)
        {
            this.op = "service_response";
            this.service = service;
            this.values = values;
            result = Result;
        }
    }
    internal class ServiceAdvertisement : Communication
    {
        public string type { get; set; } // required
        public string service { get; set; } // required

        internal ServiceAdvertisement(string service, string type) 
        {
            this.op = "advertise_service";
            this.service = service;
            this.type = type;
        }
    }
    internal class ServiceUnadvertisement : Communication
    {
        public string service { get; set; } // required

        internal ServiceUnadvertisement(string Service)
        {
            this.op = "unadvertise_service";
            service = Service;
        }
    }

    #if ROS2 
    #region Action

    // Provider side
    internal class ActionAdvertisement : Communication
    {
        public string type { get; set; } // required, message type of the advertised action
        public string action { get; set; } // required, name of the action to advertise

        internal ActionAdvertisement(string action, string type)
        {
            this.op = "advertise_action";
            this.type = type;
            this.action = action;
        }
    }

    // Provider side
    internal class ActionUnadvertisement : Communication
    {
        public string action { get; set; } // required

        internal ActionUnadvertisement(string action)
        {
            this.op = "unadvertise_action";
            this.action = action;
        }
    }
    
    // Consumer side
    internal class SendActionGoal<T> : Communication where T : Message // Message is the auto generated action goal message
    {
        public string action { get; set; } // required, the name of the action to send a goal to
        public string action_type { get; set; } // required, the action message type
        public T args { get; set; } // optional, list of json objects representing the arguments to the service
        public bool feedback { get; set; } // optional, if true, sends feedback messages over rosbridge. Defaults to false.
        public int fragment_size { get; set; } // optional, maximum size that the result and feedback messages can take before they are fragmented
        public string compression {  get; set; } // optional, an optional string to specify the compression scheme to be used on messages. Valid values are "none" and "png"

        internal SendActionGoal(string id, string action, string action_type, T args, bool feedback = false, int fragment_size = int.MaxValue, string compression = "none") : base(id)
        {
            this.op = "send_action_goal";
            this.id = id;
            this.action = action;
            this.action_type = action_type;
            this.args = args;
            this.feedback = feedback;
            this.fragment_size = fragment_size;
            this.compression = compression;
        }
    }

    // Consumer Side
    internal class CancelActionGoal : Communication
    {
        public string action { get; set; } // required
        
        internal CancelActionGoal(string id, string frameId, string action) : base(id) 
        {
            this.op = "cancel_action_goal";
            this.id = frameId;  // The ID of the goal to cancel, needs to match the ID of the goal that was sent
            this.action = action;
        }
    }

    // Provider side
    internal class ActionFeedbackResponse<T> : Communication where T : Message
    {
        public string action { get; set; } // required
        public T values { get; set; } // (values) required

        internal ActionFeedbackResponse(string id, string action, T values) : base(id) // id must match an already in-progress goal
        {
            this.op = "action_feedback";
            this.id = id;
            this.action = action;
            this.values = values;
        }
    }

    // Provider side
    internal class ActionResultResponse<T> : Communication where T : Message
    {
        public string action { get; set; } // required
        public T values { get; set; } // required, if the service had no return values, then this field can be omitted (and will be by the rosbridge server)
        public int status { get; set; } // required, return status of the action. This matches the enumeration in the action_msgs/msg/GoalStatus
        public bool result { get; set; } // required, return value of action. True means success, false failure.


        internal ActionResultResponse(string id, string action, T values, int status, bool result) : base(id) // if an ID was provided to the action goal, then the action result will contain the ID
        {
            this.op = "action_result";
            this.action = action;
            this.id = id;
            this.values = values;
            this.status = status;
            this.result = result;
        }
    }


    #endregion
    #endif
}
