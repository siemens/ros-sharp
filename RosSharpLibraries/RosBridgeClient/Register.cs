
using System;
using System.Collections.Generic;

namespace RosSharp.RosBridgeClient
{
    public class Register
    {
        private Dictionary<string, Operator> _dict = new Dictionary<string, Operator>();

        public void Add<T>(string key, T value) where T : Operator
        {
            _dict.Add(key, value);
        }

        public T GetValue<T>(string key) where T : Operator
        {
            return _dict[key] as T;
        }

        public string GetName(string key)
        {
            return _dict[key].Name;
        }
        public Type GetMessageType(string key)
        {
            return _dict[key].MessageType;
        }

        public void Remove(string key)
        {
            _dict.Remove(key);
        }

        public int Count { get { return _dict.Count; } }
    }
}
