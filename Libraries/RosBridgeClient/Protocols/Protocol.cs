using System;
using System.Threading;

namespace RosSharp.RosBridgeClient.Protocols
{
    public abstract class Protocol
    {
        private const int millisecondsCheckConnection = 100;

        public abstract void Connect();
        public abstract void Close();
        public abstract bool IsAlive();
        public abstract void Send(byte[] data);

        public abstract event EventHandler OnReceive;

        public bool WaitForConnection(int timeout)
        {
            System.Diagnostics.Stopwatch stopwatch = new System.Diagnostics.Stopwatch();
            stopwatch.Start();

            while (IsAlive() && stopwatch.Elapsed < TimeSpan.FromSeconds(timeout))
                Thread.Sleep(millisecondsCheckConnection);

            stopwatch.Stop();
            if (IsAlive())
                return true;

            return false;
        }
    }
}
