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

using Newtonsoft.Json;
using System;
using System.Text;
using System.IO;
using Newtonsoft.Json.Linq;

namespace RosSharp.RosBridgeClient.Serializers
{
    public class JsonNetSerializer : ISerializer
    {
        JsonSerializer serializer = new JsonSerializer();


        public T Deserialize<T>(byte[] bytes)
        {
            return JsonConvert.DeserializeObject<T>(Encoding.ASCII.GetString(bytes));
        }

        public T Deserialize<T>(ArraySegment<byte> bytes)
        {
            MemoryStream ms = new MemoryStream(bytes.Array, bytes.Offset, bytes.Count);
            StreamReader reader = new StreamReader(ms, Encoding.UTF8);
            JsonTextReader jreader = new JsonTextReader(reader);
            return serializer.Deserialize<T>(jreader);
        }

        public IReceivedMessage DeserializeReceived(byte[] bytes)
        {
            var jObject = Deserialize<JObject>(bytes);
            return new JsonNetReceivedMessage(jObject);
        }

        public IReceivedMessage DeserializeReceived(ArraySegment<byte> bytes)
        {
            var jObject = Deserialize<JObject>(bytes);
            return new JsonNetReceivedMessage(jObject);
        }

        public byte[] Serialize<T>(T communication)
        {
            return Encoding.ASCII.GetBytes(JsonConvert.SerializeObject(communication));
        }

        public ArraySegment<byte> SerializeUnsafe<T>(T communication)
        {
            var buffer = MemoryPool.GetBuffer();
            string ascii = JsonConvert.SerializeObject(communication);
            int bytesLength = Encoding.ASCII.GetBytes(ascii, 0, ascii.Length, buffer, 0);
            return new ArraySegment<byte>(buffer, 0, bytesLength);
        }

        public string GetJsonString(byte[] bytes)
        {
            string ascii = Encoding.ASCII.GetString(bytes);
            JObject jObject = JsonConvert.DeserializeObject<JObject>(ascii);
            return JsonConvert.SerializeObject(jObject, Formatting.Indented);
        }

    }

}
