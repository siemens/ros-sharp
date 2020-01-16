/*
© University of Kent, 2019
Author: Odysseas Doumas <od79@kent.ac.uk>
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

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
