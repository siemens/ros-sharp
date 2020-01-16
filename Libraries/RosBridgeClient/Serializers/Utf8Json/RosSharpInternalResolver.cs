#pragma warning disable 618
#pragma warning disable 612
#pragma warning disable 414
#pragma warning disable 168

namespace Utf8Json.Resolvers
{
    using System;
    using System.Reflection;

    public class RosSharpInternalResolver : global::Utf8Json.IJsonFormatterResolver
    {
        public static readonly global::Utf8Json.IJsonFormatterResolver Instance = new RosSharpInternalResolver();

        RosSharpInternalResolver()
        {

        }

        public global::Utf8Json.IJsonFormatter<T> GetFormatter<T>()
        {
            return FormatterCache<T>.formatter;
        }

        static class FormatterCache<T>
        {
            public static readonly global::Utf8Json.IJsonFormatter<T> formatter;

            static FormatterCache()
            {
                var f = RosSharpInternalResolverGenGetFormatterHelper.GetFormatter(typeof(T));
                if (f != null)
                {
                    formatter = (global::Utf8Json.IJsonFormatter<T>)f;
                }
            }
        }
    }

    internal static class RosSharpInternalResolverGenGetFormatterHelper
    {
        static readonly global::System.Collections.Generic.Dictionary<Type, int> lookup;

        static RosSharpInternalResolverGenGetFormatterHelper()
        {
            lookup = new global::System.Collections.Generic.Dictionary<Type, int>(14)
            {
                {typeof(global::RosSharp.RosBridgeClient.Communication), 0 },
                {typeof(global::RosSharp.RosBridgeClient.Advertisement), 1 },
                {typeof(global::RosSharp.RosBridgeClient.Unadvertisement), 2 },
                {typeof(global::RosSharp.RosBridgeClient.Publication<>), 3 },
                {typeof(global::RosSharp.RosBridgeClient.Subscription), 4 },
                {typeof(global::RosSharp.RosBridgeClient.Unsubscription), 5 },
                {typeof(global::RosSharp.RosBridgeClient.ServiceCall<>), 6 },
                {typeof(global::RosSharp.RosBridgeClient.ServiceResponse<>), 7 },
                {typeof(global::RosSharp.RosBridgeClient.ServiceAdvertisement), 8 },
                {typeof(global::RosSharp.RosBridgeClient.ServiceUnadvertisement), 9 },
                {typeof(global::RosSharp.RosBridgeClient.Serializers.Utf8JsonReceivedMessage), 10 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Std.Empty), 11 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Std.TriggerRequest), 12 }
            };
        }

        internal static object GetFormatter(Type t)
        {
            int key;
            var ti = t.GetTypeInfo();

            if (ti.IsGenericType)
            {
                var genericType = ti.GetGenericTypeDefinition();
                if (!lookup.TryGetValue(genericType, out key)) return null;

                switch (key)
                {
                    case 3: return CreateInstance(typeof(Utf8Json.Formatters.RosSharp.RosBridgeClient.PublicationFormatter<>), ti.GenericTypeArguments);
                    case 6: return CreateInstance(typeof(Utf8Json.Formatters.RosSharp.RosBridgeClient.ServiceCallFormatter<>), ti.GenericTypeArguments);
                    case 7: return CreateInstance(typeof(Utf8Json.Formatters.RosSharp.RosBridgeClient.ServiceResponseFormatter<>), ti.GenericTypeArguments);
                    default: return null;
                }
            }
            else
            {
                if (!lookup.TryGetValue(t, out key)) return null;
                switch (key)
                {
                    case 0: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.CommunicationFormatter();
                    case 1: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.AdvertisementFormatter();
                    case 2: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.UnadvertisementFormatter();
                    case 4: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.SubscriptionFormatter();
                    case 5: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.UnsubscriptionFormatter();
                    case 8: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.ServiceAdvertisementFormatter();
                    case 9: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.ServiceUnadvertisementFormatter();
                    case 10: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.Utf8JsonReceivedMessageFormatter();
                    case 11: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.EmptyFormatter();
                    case 12: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.TriggerRequestFormatter();
                    default:
                        return null;
                }
            }
        }

        static object CreateInstance(Type genericType, Type[] genericTypeArguments, params object[] arguments)
        {
            return Activator.CreateInstance(genericType.MakeGenericType(genericTypeArguments));
            //return Activator.CreateInstance(genericType.MakeGenericType(genericTypeArguments), true);
        }

    }
}

#pragma warning disable 168
#pragma warning restore 414
#pragma warning restore 618
#pragma warning restore 612

#pragma warning disable 618
#pragma warning disable 612
#pragma warning disable 414
#pragma warning disable 219
#pragma warning disable 168

namespace Utf8Json.Formatters.RosSharp.RosBridgeClient
{
    using System;
    using Utf8Json;


    internal sealed class CommunicationFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.Communication>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public CommunicationFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("op"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("id"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("op"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("id"),

            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.Communication value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }

            JsonSerializer.NonGeneric.Serialize(ref writer, value, formatterResolver);
        }

        public global::RosSharp.RosBridgeClient.Communication Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }


            throw new InvalidOperationException("generated serializer for IInterface does not support deserialize.");
        }
    }


    internal sealed class AdvertisementFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.Advertisement>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public AdvertisementFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("topic"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("type"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("op"), 2},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("id"), 3},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("topic"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("type"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("op"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("id"),

            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.Advertisement value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }


            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.topic);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteString(value.type);
            writer.WriteRaw(this.____stringByteKeys[2]);
            writer.WriteString(value.op);
            writer.WriteRaw(this.____stringByteKeys[3]);
            writer.WriteString(value.id);

            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.Advertisement Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }


            var __topic__ = default(string);
            var __topic__b__ = false;
            var __type__ = default(string);
            var __type__b__ = false;
            var __op__ = default(string);
            var __op__b__ = false;
            var __id__ = default(string);
            var __id__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __topic__ = reader.ReadString();
                        __topic__b__ = true;
                        break;
                    case 1:
                        __type__ = reader.ReadString();
                        __type__b__ = true;
                        break;
                    case 2:
                        __op__ = reader.ReadString();
                        __op__b__ = true;
                        break;
                    case 3:
                        __id__ = reader.ReadString();
                        __id__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

            NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.Advertisement(__id__, __topic__, __type__);
            if (__topic__b__) ____result.topic = __topic__;
            if (__type__b__) ____result.type = __type__;
            if (__op__b__) ____result.op = __op__;
            if (__id__b__) ____result.id = __id__;

            return ____result;
        }
    }


    internal sealed class UnadvertisementFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.Unadvertisement>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public UnadvertisementFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("topic"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("op"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("id"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("topic"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("op"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("id"),

            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.Unadvertisement value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }


            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.topic);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteString(value.op);
            writer.WriteRaw(this.____stringByteKeys[2]);
            writer.WriteString(value.id);

            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.Unadvertisement Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }


            var __topic__ = default(string);
            var __topic__b__ = false;
            var __op__ = default(string);
            var __op__b__ = false;
            var __id__ = default(string);
            var __id__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __topic__ = reader.ReadString();
                        __topic__b__ = true;
                        break;
                    case 1:
                        __op__ = reader.ReadString();
                        __op__b__ = true;
                        break;
                    case 2:
                        __id__ = reader.ReadString();
                        __id__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

            NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.Unadvertisement(__id__, __topic__);
            if (__topic__b__) ____result.topic = __topic__;
            if (__op__b__) ____result.op = __op__;
            if (__id__b__) ____result.id = __id__;

            return ____result;
        }
    }


    internal sealed class PublicationFormatter<T> : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.Publication<T>> where T : global::RosSharp.RosBridgeClient.Message, new()
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public PublicationFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("topic"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("msg"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("op"), 2},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("id"), 3},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("topic"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("msg"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("op"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("id"),

            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.Publication<T> value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }


            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.topic);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<T>().Serialize(ref writer, value.msg, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[2]);
            writer.WriteString(value.op);
            writer.WriteRaw(this.____stringByteKeys[3]);
            writer.WriteString(value.id);

            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.Publication<T> Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }

            var __topic__ = default(string);
            var __topic__b__ = false;
            var __msg__ = default(T);
            var __msg__b__ = false;
            var __op__ = default(string);
            var __op__b__ = false;
            var __id__ = default(string);
            var __id__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __topic__ = reader.ReadString();
                        __topic__b__ = true;
                        break;
                    case 1:
                        __msg__ = formatterResolver.GetFormatterWithVerify<T>().Deserialize(ref reader, formatterResolver);
                        __msg__b__ = true;
                        break;
                    case 2:
                        __op__ = reader.ReadString();
                        __op__b__ = true;
                        break;
                    case 3:
                        __id__ = reader.ReadString();
                        __id__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

            NEXT_LOOP:
                continue;
            }

            return new global::RosSharp.RosBridgeClient.Publication<T>(__id__, __topic__, __msg__);
        }
    }


    internal sealed class SubscriptionFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.Subscription>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public SubscriptionFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("topic"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("type"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("throttle_rate"), 2},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("queue_length"), 3},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("fragment_size"), 4},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("compression"), 5},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("op"), 6},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("id"), 7},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("topic"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("type"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("throttle_rate"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("queue_length"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("fragment_size"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("compression"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("op"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("id"),

            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.Subscription value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }


            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.topic);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteString(value.type);
            writer.WriteRaw(this.____stringByteKeys[2]);
            writer.WriteInt32(value.throttle_rate);
            writer.WriteRaw(this.____stringByteKeys[3]);
            writer.WriteInt32(value.queue_length);
            writer.WriteRaw(this.____stringByteKeys[4]);
            writer.WriteInt32(value.fragment_size);
            writer.WriteRaw(this.____stringByteKeys[5]);
            writer.WriteString(value.compression);
            writer.WriteRaw(this.____stringByteKeys[6]);
            writer.WriteString(value.op);
            writer.WriteRaw(this.____stringByteKeys[7]);
            writer.WriteString(value.id);

            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.Subscription Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }


            var __topic__ = default(string);
            var __topic__b__ = false;
            var __type__ = default(string);
            var __type__b__ = false;
            var __throttle_rate__ = default(int);
            var __throttle_rate__b__ = false;
            var __queue_length__ = 1;
            var __queue_length__b__ = false;
            var __fragment_size__ = int.MaxValue;
            var __fragment_size__b__ = false;
            var __compression__ = "none";
            var __compression__b__ = false;
            var __op__ = default(string);
            var __op__b__ = false;
            var __id__ = default(string);
            var __id__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __topic__ = reader.ReadString();
                        __topic__b__ = true;
                        break;
                    case 1:
                        __type__ = reader.ReadString();
                        __type__b__ = true;
                        break;
                    case 2:
                        __throttle_rate__ = reader.ReadInt32();
                        __throttle_rate__b__ = true;
                        break;
                    case 3:
                        __queue_length__ = reader.ReadInt32();
                        __queue_length__b__ = true;
                        break;
                    case 4:
                        __fragment_size__ = reader.ReadInt32();
                        __fragment_size__b__ = true;
                        break;
                    case 5:
                        __compression__ = reader.ReadString();
                        __compression__b__ = true;
                        break;
                    case 6:
                        __op__ = reader.ReadString();
                        __op__b__ = true;
                        break;
                    case 7:
                        __id__ = reader.ReadString();
                        __id__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

            NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.Subscription(__id__, __topic__, __type__, __throttle_rate__, __queue_length__, __fragment_size__, __compression__);
            if (__topic__b__) ____result.topic = __topic__;
            if (__type__b__) ____result.type = __type__;
            if (__throttle_rate__b__) ____result.throttle_rate = __throttle_rate__;
            if (__queue_length__b__) ____result.queue_length = __queue_length__;
            if (__fragment_size__b__) ____result.fragment_size = __fragment_size__;
            if (__compression__b__) ____result.compression = __compression__;
            if (__op__b__) ____result.op = __op__;
            if (__id__b__) ____result.id = __id__;

            return ____result;
        }
    }


    internal sealed class UnsubscriptionFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.Unsubscription>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public UnsubscriptionFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("topic"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("op"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("id"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("topic"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("op"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("id"),

            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.Unsubscription value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }


            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.topic);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteString(value.op);
            writer.WriteRaw(this.____stringByteKeys[2]);
            writer.WriteString(value.id);

            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.Unsubscription Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }


            var __topic__ = default(string);
            var __topic__b__ = false;
            var __op__ = default(string);
            var __op__b__ = false;
            var __id__ = default(string);
            var __id__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __topic__ = reader.ReadString();
                        __topic__b__ = true;
                        break;
                    case 1:
                        __op__ = reader.ReadString();
                        __op__b__ = true;
                        break;
                    case 2:
                        __id__ = reader.ReadString();
                        __id__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

            NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.Unsubscription(__id__, __topic__);
            if (__topic__b__) ____result.topic = __topic__;
            if (__op__b__) ____result.op = __op__;
            if (__id__b__) ____result.id = __id__;

            return ____result;
        }
    }


    internal sealed class ServiceCallFormatter<T> : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.ServiceCall<T>>
        where T : global::RosSharp.RosBridgeClient.Message, new()
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public ServiceCallFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("service"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("args"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("fragment_size"), 2},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("compression"), 3},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("op"), 4},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("id"), 5},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("service"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("args"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("fragment_size"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("compression"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("op"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("id"),

            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.ServiceCall<T> value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }


            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.service);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<T>().Serialize(ref writer, value.args, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[2]);
            writer.WriteInt32(value.fragment_size);
            writer.WriteRaw(this.____stringByteKeys[3]);
            writer.WriteString(value.compression);
            writer.WriteRaw(this.____stringByteKeys[4]);
            writer.WriteString(value.op);
            writer.WriteRaw(this.____stringByteKeys[5]);
            writer.WriteString(value.id);

            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.ServiceCall<T> Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }

            var __service__ = default(string);
            var __service__b__ = false;
            var __args__ = default(T);
            var __args__b__ = false;
            var __fragment_size__ = int.MaxValue;
            var __fragment_size__b__ = false;
            var __compression__ = "none";
            var __compression__b__ = false;
            var __op__ = default(string);
            var __op__b__ = false;
            var __id__ = default(string);
            var __id__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __service__ = reader.ReadString();
                        __service__b__ = true;
                        break;
                    case 1:
                        __args__ = formatterResolver.GetFormatterWithVerify<T>().Deserialize(ref reader, formatterResolver);
                        __args__b__ = true;
                        break;
                    case 2:
                        __fragment_size__ = reader.ReadInt32();
                        __fragment_size__b__ = true;
                        break;
                    case 3:
                        __compression__ = reader.ReadString();
                        __compression__b__ = true;
                        break;
                    case 4:
                        __op__ = reader.ReadString();
                        __op__b__ = true;
                        break;
                    case 5:
                        __id__ = reader.ReadString();
                        __id__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

            NEXT_LOOP:
                continue;
            }

            return new global::RosSharp.RosBridgeClient.ServiceCall<T>(__id__, __service__, __args__, __fragment_size__, __compression__);
        }

    }


    internal sealed class ServiceResponseFormatter<T> : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.ServiceResponse<T>>
        where T : global::RosSharp.RosBridgeClient.Message, new()
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public ServiceResponseFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("service"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("values"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("result"), 2},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("op"), 3},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("id"), 4},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("service"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("values"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("result"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("op"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("id"),

            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.ServiceResponse<T> value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }


            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.service);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<T>().Serialize(ref writer, value.values, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[2]);
            writer.WriteBoolean(value.result);
            writer.WriteRaw(this.____stringByteKeys[3]);
            writer.WriteString(value.op);
            writer.WriteRaw(this.____stringByteKeys[4]);
            writer.WriteString(value.id);

            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.ServiceResponse<T> Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }

            var __service__ = default(string);
            var __service__b__ = false;
            var __values__ = default(T);
            var __values__b__ = false;
            var __result__ = default(bool);
            var __result__b__ = false;
            var __op__ = default(string);
            var __op__b__ = false;
            var __id__ = default(string);
            var __id__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __service__ = reader.ReadString();
                        __service__b__ = true;
                        break;
                    case 1:
                        __values__ = formatterResolver.GetFormatterWithVerify<T>().Deserialize(ref reader, formatterResolver);
                        __values__b__ = true;
                        break;
                    case 2:
                        __result__ = reader.ReadBoolean();
                        __result__b__ = true;
                        break;
                    case 3:
                        __op__ = reader.ReadString();
                        __op__b__ = true;
                        break;
                    case 4:
                        __id__ = reader.ReadString();
                        __id__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

            NEXT_LOOP:
                continue;
            }

            return new global::RosSharp.RosBridgeClient.ServiceResponse<T>(__id__, __service__, __values__, __result__);
        }
    }


    internal sealed class ServiceAdvertisementFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.ServiceAdvertisement>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public ServiceAdvertisementFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("type"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("service"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("op"), 2},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("id"), 3},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("type"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("service"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("op"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("id"),

            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.ServiceAdvertisement value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }


            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.type);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteString(value.service);
            writer.WriteRaw(this.____stringByteKeys[2]);
            writer.WriteString(value.op);
            writer.WriteRaw(this.____stringByteKeys[3]);
            writer.WriteString(value.id);

            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.ServiceAdvertisement Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }


            var __type__ = default(string);
            var __type__b__ = false;
            var __service__ = default(string);
            var __service__b__ = false;
            var __op__ = default(string);
            var __op__b__ = false;
            var __id__ = default(string);
            var __id__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __type__ = reader.ReadString();
                        __type__b__ = true;
                        break;
                    case 1:
                        __service__ = reader.ReadString();
                        __service__b__ = true;
                        break;
                    case 2:
                        __op__ = reader.ReadString();
                        __op__b__ = true;
                        break;
                    case 3:
                        __id__ = reader.ReadString();
                        __id__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

            NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.ServiceAdvertisement(__service__, __type__);
            if (__type__b__) ____result.type = __type__;
            if (__service__b__) ____result.service = __service__;
            if (__op__b__) ____result.op = __op__;
            if (__id__b__) ____result.id = __id__;

            return ____result;
        }
    }


    internal sealed class ServiceUnadvertisementFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.ServiceUnadvertisement>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public ServiceUnadvertisementFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("service"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("op"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("id"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("service"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("op"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("id"),

            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.ServiceUnadvertisement value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }


            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.service);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteString(value.op);
            writer.WriteRaw(this.____stringByteKeys[2]);
            writer.WriteString(value.id);

            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.ServiceUnadvertisement Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }


            var __service__ = default(string);
            var __service__b__ = false;
            var __op__ = default(string);
            var __op__b__ = false;
            var __id__ = default(string);
            var __id__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __service__ = reader.ReadString();
                        __service__b__ = true;
                        break;
                    case 1:
                        __op__ = reader.ReadString();
                        __op__b__ = true;
                        break;
                    case 2:
                        __id__ = reader.ReadString();
                        __id__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

            NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.ServiceUnadvertisement(__service__);
            if (__service__b__) ____result.service = __service__;
            if (__op__b__) ____result.op = __op__;
            if (__id__b__) ____result.id = __id__;

            return ____result;
        }
    }


    // Customized
    internal sealed class Utf8JsonReceivedMessageFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.Serializers.Utf8JsonReceivedMessage>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly global::Utf8Json.Internal.AutomataDictionary ____op_keyMapping;
        readonly byte[][] ____stringByteKeys;

        [ThreadStatic]
        private static readonly int[] ____id_offset = new int[3];

        public Utf8JsonReceivedMessageFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("op"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("topic"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("service"), 2},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("id"), 3},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("msg"), 4},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("args"), 5},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("values"), 6},
            };

            this.____op_keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("publish"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("call_service"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("service_response"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("op"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("topic"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("service"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("id"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("msg"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("args"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("values"),
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.Serializers.Utf8JsonReceivedMessage value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            throw new InvalidOperationException("Received message should not be serialized. Please use the corresponding communications classes.");
        }

        public global::RosSharp.RosBridgeClient.Serializers.Utf8JsonReceivedMessage Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return default(global::RosSharp.RosBridgeClient.Serializers.Utf8JsonReceivedMessage);
            }

            var __op__ = default(string);
            var __op__b__ = false;
            var __topic__ = default(string);
            var __topic__b__ = false;
            var __service__ = default(string);
            var __service__b__ = false;
            var __id__ = default(string);
            var __id__b__ = false;

            var ____op__key = -1;
            var ____count = 0;

            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        var op_segment = reader.ReadStringSegmentRaw();
                        ____op_keyMapping.TryGetValueSafe(op_segment, out ____op__key);
                        __op__ = System.Text.Encoding.UTF8.GetString(op_segment.Array, op_segment.Offset, op_segment.Count);
                        __op__b__ = true;
                        break;
                    case 1:
                        __topic__ = reader.ReadString();
                        __topic__b__ = true;
                        break;
                    case 2:
                        __service__ = reader.ReadString();
                        __service__b__ = true;
                        break;
                    case 3:
                        __id__ = reader.ReadString();
                        __id__b__ = true;
                        break;
                    case 4:
                    case 5:
                    case 6:
                        ____id_offset[key - 4] = reader.GetCurrentOffsetUnsafe();
                        reader.ReadNextBlock();
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

            NEXT_LOOP:
                continue;
            }

            switch (____op__key)
            {
                case 0:
                case 1:
                case 2:
                    return new global::RosSharp.RosBridgeClient.Serializers.Utf8JsonReceivedMessage(__op__, __topic__, __service__, __id__, ____id_offset[____op__key]);
                default:
                    break;
            }

            return new global::RosSharp.RosBridgeClient.Serializers.Utf8JsonReceivedMessage(__op__, __topic__, __service__, __id__, -1);
        }
    }


    public sealed class EmptyFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Std.Empty>
    {
        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Std.Empty value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            writer.WriteBeginObject();
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Std.Empty Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            reader.ReadIsBeginObjectWithVerify();
            var ____count = 0;
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                reader.ReadNext();
            }
            return new global::RosSharp.RosBridgeClient.MessageTypes.Std.Empty();
        }
    }


    public sealed class TriggerRequestFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Std.TriggerRequest>
    {
        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Std.TriggerRequest value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            writer.WriteBeginObject();
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Std.TriggerRequest Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            reader.ReadIsBeginObjectWithVerify();
            var ____count = 0;
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                reader.ReadNext();
            }
            return new global::RosSharp.RosBridgeClient.MessageTypes.Std.TriggerRequest();
        }
    }

}

#pragma warning disable 168
#pragma warning restore 219
#pragma warning restore 414
#pragma warning restore 618
#pragma warning restore 612
