using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RosSharp.RosBridgeClient.Serializers
{
    public interface ISerializer
    {
        byte[] Serialize<T>(T communication);
        ArraySegment<byte> SerializeUnsafe<T>(T communication);

        IReceivedMessage DeserializeReceived(byte[] bytes);
        IReceivedMessage DeserializeReceived(ArraySegment<byte> bytes);

        string GetJsonString(byte[] bytes);
    }
}
