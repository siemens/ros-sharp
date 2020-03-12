//using System;
//using System.Collections.Generic;
//using System.Linq;
//using System.Text;
//using System.Threading.Tasks;
//using System.Text.Json;

//namespace RosBridgeClient
//{
//    public class Serializer
//    {

//        public virtual byte[] Serialize<T>(T obj)
//        {
//            switch (Serializer)
//            {
//                case SerializerEnum.JSON:
//                    string json = JsonSerializer.Serialize(obj, obj.GetType());
//                    return Encoding.ASCII.GetBytes(json);
//                //case SerializerEnum.BSON:
//                //    System.IO.MemoryStream ms = new System.IO.MemoryStream();
//                //    Newtonsoft.Json.Bson.BsonDataWriter writer = new Newtonsoft.Json.Bson.BsonDataWriter(ms);
//                //    Newtonsoft.Json.JsonSerializer serializer = new Newtonsoft.Json.JsonSerializer();
//                //    serializer.Serialize(writer, obj);
//                //    return ms.ToArray();
//                default:
//                    throw new ArgumentException("Invalid Serializer");
//            }
//        }

//        public virtual object Deserialize(byte[] buffer)
//        {
//            switch (Serializer)
//            {
//                case SerializerEnum.JSON:
//                    string ascii = Encoding.ASCII.GetString(buffer, 0, buffer.Length);
//                    return JsonDocument.Parse(ascii).RootElement;
//                //case SerializerEnum.BSON:
//                //    System.IO.MemoryStream ms = new System.IO.MemoryStream(buffer);
//                //    Newtonsoft.Json.Bson.BsonDataReader reader = new Newtonsoft.Json.Bson.BsonDataReader(ms);
//                //    return new Newtonsoft.Json.JsonSerializer().Deserialize<T>(reader);
//                default:
//                    throw new ArgumentException("Invalid Serializer");
//            }
//        }
//    }
//}
