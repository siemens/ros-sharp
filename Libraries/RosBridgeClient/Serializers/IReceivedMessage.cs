using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RosSharp.RosBridgeClient.Serializers
{
    public interface IReceivedMessage
    {
        string Op { get; }
        string Id { get; }
        string Topic { get; }
        string Service { get; }
        T GetArgs<T>() where T : Message;
        T GetMessage<T>() where T : Message;
        T GetValues<T>() where T : Message;
    }
}
