using System;

namespace RosSharp.RosBridgeClient.Serializers
{
    internal static class MemoryPool
    {
        [ThreadStatic]
        static byte[] buffer = null;

        public const int Size = 65536;

        public static byte[] GetBuffer()
        {
            if (buffer == null)
            {
                buffer = new byte[Size];
            }
            return buffer;
        }
    }
}
