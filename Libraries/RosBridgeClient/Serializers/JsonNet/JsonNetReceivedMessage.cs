using Newtonsoft.Json.Linq;

namespace RosSharp.RosBridgeClient.Serializers
{
    internal class JsonNetReceivedMessage : IReceivedMessage
    {
        public readonly JObject data;

        internal JsonNetReceivedMessage(JObject data)
        {
            this.data = data;
        }

        public string Op => get("op");
        public string Id => get("id");
        public string Topic => get("topic");
        public string Service => get("service");

        private string get(string id)
        {
            var val = data.GetValue(id);
            if (val == null)
            {
                return null;
            }
            return val.ToString();
        }

        private T getObject<T>(string id)
        {
            var val = data.GetValue(id);
            if (val == null)
            {
                return default(T);
            }
            return val.ToObject<T>();
        }

        public T GetArgs<T>() where T : Message
        {
            return getObject<T>("args");
        }

        public T GetMessage<T>() where T : Message
        {
            return getObject<T>("msg");
        }

        public T GetValues<T>() where T : Message
        {
            return getObject<T>("values");
        }
    }
}
