using System;
using System.Reflection;
using Utf8Json;
using Utf8Json.Formatters.RosSharp.RosBridgeClient;
using Utf8Json.Resolvers;

namespace RosSharp.RosBridgeClient.Serializers
{
    /// <summary>
    /// Utf8Json serializer implementation. Consumers of RosSharp MUST setup utf8json manualy
    /// RosSharp provides :
    /// RosSharpResolverGen for messages and services,
    /// RosSharpInternalResolver for internal rosbridge communications,
    /// ByteArrayToBase64Formatter and ByteArraySegmentToBase64Formatter for zero allocation conversion of byte arrays
    /// Users shoudl also use EnumResolver.UnderlyingValue
    /// </summary>
    public class Utf8JsonSerializer : ISerializer
    {
        public T Deserialize<T>(byte[] bytes)
        {
            return JsonSerializer.Deserialize<T>(bytes, RosSharpDefaultResolver.Instance);
        }

        public T Deserialize<T>(ArraySegment<byte> bytes)
        {
            return JsonSerializer.Deserialize<T>(bytes.Array, bytes.Offset, RosSharpDefaultResolver.Instance);
        }

        public IReceivedMessage DeserializeReceived(byte[] bytes)
        {
            Utf8JsonReceivedMessage commInfo = Deserialize<Utf8JsonReceivedMessage>(bytes);
            commInfo.buffer = bytes;
            return commInfo;
        }

        public IReceivedMessage DeserializeReceived(ArraySegment<byte> bytes)
        {
            Utf8JsonReceivedMessage commInfo = Deserialize<Utf8JsonReceivedMessage>(bytes);
            commInfo.buffer = bytes.Array;
            return commInfo;
        }

        public byte[] Serialize<T>(T communication)
        {
            return JsonSerializer.Serialize(communication, RosSharpDefaultResolver.Instance);
        }

        public ArraySegment<byte> SerializeUnsafe<T>(T communication)
        {
            return JsonSerializer.SerializeUnsafe(communication, RosSharpDefaultResolver.Instance);
        }

        public string GetJsonString(byte[] bytes)
        {
            return System.Text.Encoding.UTF8.GetString(JsonSerializer.PrettyPrintByteArray(bytes));
        }

    }

    public class RosSharpDefaultResolver : IJsonFormatterResolver
    {
        public static IJsonFormatterResolver Instance = new RosSharpDefaultResolver();

        // configure your resolver and formatters.
        static IJsonFormatter[] formatters = new IJsonFormatter[]{
            new ByteArrayToBase64Formatter(),
            new ByteArraySegmentToBase64Formatter(),

            //For some reason AOT works fine with this here so we'll keep it for now
            new PublicationFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Std.String>()
    };

        static readonly IJsonFormatterResolver[] resolvers = new[]
        {
            RosSharpInternalResolver.Instance,
            RosSharpResolverGen.Instance,

            BuiltinResolver.Instance,
            EnumResolver.UnderlyingValue,
            DynamicGenericResolver.Instance
        };

        RosSharpDefaultResolver()
        {
        }

        public IJsonFormatter<T> GetFormatter<T>()
        {
            return FormatterCache<T>.formatter;
        }

        static class FormatterCache<T>
        {
            public static readonly IJsonFormatter<T> formatter;

            static FormatterCache()
            {
                foreach (var item in formatters)
                {
                    foreach (var implInterface in item.GetType().GetTypeInfo().ImplementedInterfaces)
                    {
                        var ti = implInterface.GetTypeInfo();
                        if (ti.IsGenericType && ti.GenericTypeArguments[0] == typeof(T))
                        {
                            formatter = (IJsonFormatter<T>)item;
                            return;
                        }
                    }
                }

                foreach (var item in resolvers)
                {
                    var f = item.GetFormatter<T>();
                    if (f != null)
                    {
                        formatter = f;
                        return;
                    }
                }
            }
        }
    }
}
