/*
© Siemens AG, 2020
Author: Berkay Alp Cakal (berkay_alp.cakal.ct@siemens.com)

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

using System.Text;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace RosSharp.RosBridgeClient
{
    internal class NewtonsoftSerializer : ISerializer
    {
        public bool UseJSON { get; set; }

        public NewtonsoftSerializer(bool useJSON = true)
        {
            UseJSON = useJSON;
        }

        public byte[] Serialize<T>(T obj)
        {
            if (UseJSON)
            {
                string json = JsonConvert.SerializeObject(obj);
                return Encoding.ASCII.GetBytes(json);
            }
            else
            {
                System.IO.MemoryStream ms = new System.IO.MemoryStream();
                Newtonsoft.Json.Bson.BsonDataWriter writer = new Newtonsoft.Json.Bson.BsonDataWriter(ms);
                JsonSerializer serializer = new JsonSerializer();
                serializer.Serialize(writer, obj);
                return ms.ToArray();
            }
        }

        public DeserializedObject Deserialize(byte[] buffer)
        {
            if(UseJSON)
            {
                string ascii = Encoding.ASCII.GetString(buffer, 0, buffer.Length);
                JObject jObject = JsonConvert.DeserializeObject<JObject>(ascii);
                return new NewtonsoftJsonObject(jObject);
            }
            else
            {
                System.IO.MemoryStream ms = new System.IO.MemoryStream(buffer);
                Newtonsoft.Json.Bson.BsonDataReader reader = new Newtonsoft.Json.Bson.BsonDataReader(ms);
                JObject jObject = new JsonSerializer().Deserialize<JObject>(reader);
                return new NewtonsoftJsonObject(jObject);
            }
        }
        
        public T Deserialize<T>(string json)
        {
            return JsonConvert.DeserializeObject<T>(json);
        }
    }

    internal class NewtonsoftJsonObject : DeserializedObject
    {
        private JObject jObject;

        internal NewtonsoftJsonObject(JObject _jObject)
        {
            jObject = _jObject;
        }

        internal override string GetProperty(string property)
        {
            return jObject.GetValue(property).ToString();
        }
    }
}
