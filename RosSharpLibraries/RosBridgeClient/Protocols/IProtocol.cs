using System;

namespace RosSharp.RosBridgeClient.Protocols
{
    public interface IProtocol
    {
        void Connect();
        void Close();
        bool IsAlive();
        void Send(byte[] data);
        event EventHandler OnReceive;
    }
}
