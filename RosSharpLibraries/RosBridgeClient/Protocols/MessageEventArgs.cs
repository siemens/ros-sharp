using System;


namespace RosSharp.RosBridgeClient.Protocols
{
    public class MessageEventArgs : EventArgs
    {
        public byte[] RawData;
        public MessageEventArgs(byte[] rawData)
        {
            RawData = rawData;
        }
    }
}
