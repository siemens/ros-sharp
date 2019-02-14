using System;
using Newtonsoft.Json;

namespace RosSharp.RosBridgeClient.Messages.Standard
{
    public class RosInt32 : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "std_msgs/Int32";
        public Int32 data;
        public RosInt32(Int32 data)
        {
            this.data = data;
        }
    }
}
