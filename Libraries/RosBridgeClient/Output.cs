using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RosSharp
{
    public static class Output
    {
        public static void Log(string text)
        {
            WriteHandler?.Invoke(text);
        }
        public static void LogError(string text)
        {
            if (ErrorHandler != null)
                ErrorHandler.Invoke(text);
            else
            {
                WriteHandler?.Invoke($"Error: {text}");
            }
        }

        public static void SetHandlers(Action<string> write, Action<string> error)
        {
            WriteHandler = write;
            ErrorHandler = error;
        }
        private static Action<string> WriteHandler;
        private static Action<string> ErrorHandler;
    }
}
