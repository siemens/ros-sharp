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

using System;
using System.Text;
using Utf8Json;

namespace RosSharp.RosBridgeClient.Serializers
{
    public class ByteArrayToBase64Formatter : IJsonFormatter<byte[]>
    {
        public void Serialize(ref JsonWriter writer, byte[] value, IJsonFormatterResolver formatterResolver)
        {
            if (value == null) { writer.WriteNull(); return; }

            writer.EnsureCapacity(4 * ((value.Length + 2) / 3) + 2); // unsafe, control underlying buffer manually

            writer.WriteRawUnsafe((byte)'\"');
            Base64Enc.WriteToBase64(ref writer, value);
            writer.WriteRawUnsafe((byte)'\"');
        }

        public byte[] Deserialize(ref JsonReader reader, IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull()) return null;

            string str = reader.ReadString();
            return Convert.FromBase64String(str);
        }
    }

    public class ByteArraySegmentToBase64Formatter : IJsonFormatter<ArraySegment<byte>>
    {
        public void Serialize(ref JsonWriter writer, ArraySegment<byte> value, IJsonFormatterResolver formatterResolver)
        {
            if (value.Count <= 0) { writer.WriteNull(); return; }

            writer.EnsureCapacity(4 * ((value.Count + 2) / 3) + 2); // unsafe, control underlying buffer manually

            writer.WriteRawUnsafe((byte)'\"');
            Base64Enc.WriteToBase64(ref writer, value.Array, value.Offset, value.Count);
            writer.WriteRawUnsafe((byte)'\"');
        }
        public ArraySegment<byte> Deserialize(ref JsonReader reader, IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull()) return new ArraySegment<byte>();

            string str = reader.ReadString();
            byte[] bytes = Convert.FromBase64String(str);
            return new ArraySegment<byte>(bytes);
        }

    }

    internal class Base64Enc
    {
        public static void WriteToBase64(ref JsonWriter writer, byte[] data)
        {
            WriteToBase64(ref writer, data, 0, data.Length);
        }

        public static void WriteToBase64(ref JsonWriter writer, byte[] data, int offset, int size)
        {
            //int bufferPos = 0;

            int requiredSize = 4 * ((size + 2) / 3);
            // size/76*2 for 2 line break characters    
            // if (addLineBreaks) requiredSize += requiredSize + (requiredSize / 38);

            UInt32 octet_a;
            UInt32 octet_b;
            UInt32 octet_c;
            UInt32 triple;
            //int lineCount = 0;
            int sizeMod = size - (size % 3);
            // adding all data triplets
            for (; offset < sizeMod;)
            {
                octet_a = data[offset++];
                octet_b = data[offset++];
                octet_c = data[offset++];

                triple = (octet_a << 0x10) + (octet_b << 0x08) + octet_c;

                writer.WriteRawUnsafe(base64EncodingTable[(triple >> 3 * 6) & 0x3F]);
                writer.WriteRawUnsafe(base64EncodingTable[(triple >> 2 * 6) & 0x3F]);
                writer.WriteRawUnsafe(base64EncodingTable[(triple >> 1 * 6) & 0x3F]);
                writer.WriteRawUnsafe(base64EncodingTable[(triple >> 0 * 6) & 0x3F]);
                //if (addLineBreaks)
                //{
                //    if (++lineCount == 19)
                //    {
                //        buffer[bufferPos++] = 13;
                //        buffer[bufferPos++] = 10;
                //        lineCount = 0;
                //    }
                //}
            }

            // last bytes
            if (sizeMod < size)
            {
                octet_a = offset < size ? data[offset++] : (UInt32)0;
                octet_b = offset < size ? data[offset++] : (UInt32)0;
                octet_c = (UInt32)0; // last character is definitely padded

                triple = (octet_a << 0x10) + (octet_b << 0x08) + octet_c;

                // add padding '='
                sizeMod = size % 3;

                writer.WriteRawUnsafe(base64EncodingTable[(triple >> 3 * 6) & 0x3F]);
                writer.WriteRawUnsafe(base64EncodingTable[(triple >> 2 * 6) & 0x3F]);
                //writer.WriteRawUnsafe(base64EncodingTable[(triple >> 1 * 6) & 0x3F]);
                //writer.WriteRawUnsafe(base64EncodingTable[(triple >> 0 * 6) & 0x3F]);

                if (sizeMod == 1)
                    writer.WriteRawUnsafe((byte)'=');
                else
                    writer.WriteRawUnsafe(base64EncodingTable[(triple >> 1 * 6) & 0x3F]);

                // last character is definitely padded
                writer.WriteRawUnsafe((byte)'=');
            }
        }

        public static readonly Encoding UTF8 = new UTF8Encoding(false);
        public static readonly byte[] base64EncodingTable = UTF8.GetBytes("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/");

    }

}