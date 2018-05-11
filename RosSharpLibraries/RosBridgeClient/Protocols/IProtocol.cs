using System;

namespace RosSharp.RosBridgeClient.Protocols
{
    public interface IProtocol
    {
        event EventHandler OnMessage;
        void Connect();
        void Close();
        bool IsAlive();
        void SendAsync(byte[] data, Action<bool> completed);
        
    }
}
