using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RosSharp.RosBridgeClient
{
    public static class Output
    {
        private static Action<string> Log_ = Console.WriteLine;
        public static void SetLogDelegate(Action<string> log)
        {
            Log_ = log;
        }
        public static void Log(string msg)
        {
            Log_?.Invoke(msg);
        }
    }
}
