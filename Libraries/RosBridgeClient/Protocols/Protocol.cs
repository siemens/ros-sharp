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
            return WaitForConnectionStatus(timeout, true);
        }

        public bool WaitForDisconnection(int timeout)
        {
            return WaitForConnectionStatus(timeout, false);
        }

        private bool WaitForConnectionStatus(int timeout, bool wantAlive)
        {
            System.Diagnostics.Stopwatch stopwatch = new System.Diagnostics.Stopwatch();
            stopwatch.Start();

            while ((wantAlive != IsAlive()) && stopwatch.Elapsed < TimeSpan.FromSeconds(timeout))
                Thread.Sleep(millisecondsCheckConnection);
          
            stopwatch.Stop();

            return (wantAlive == IsAlive());
        }
    }
}
