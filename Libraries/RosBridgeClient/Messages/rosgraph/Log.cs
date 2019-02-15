using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Newtonsoft.Json;

namespace RosSharp.RosBridgeClient.Messages.rosgraph
{
    public class Log : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "rosgraph_msgs/Log";
        public uint level;
        public string name;
        public string msg;
        public string file;
        public string function;
        public uint line;
        public string[] topics;
            
        public Log()
        {
        }
    }
}
