using System;
using System.Threading;

namespace RosSharp.RosBridgeClient.Protocols
{
    public interface IProtocol
    {
        void Connect();
        void Close();
        bool IsAlive();
        void Send(byte[] data);

        event EventHandler OnReceive;
        event EventHandler OnConnect;
        event EventHandler OnClose;
    }
}
