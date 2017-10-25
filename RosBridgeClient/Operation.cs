/*
© Siemens AG, 2017
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)

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

namespace RosSharp.RosBridgeClient
{
public class Operation
    {
        public virtual string op { get { return "undefined"; } } // required
        public int id; // optional

        public Operation(int Id=0)
        {
            id = Id;
        }
        public bool ShouldSerializeid() { return (id != 0); }
    }

    public class Adverisement : Operation
    {
        public override string op { get { return "advertise"; } } // required
        public string topic; // required
        public string type; // required

        public Adverisement(int Id, string Topic, string Type) : base(Id)
        {
            topic = Topic;
            type = Type;
            Id = id;
        }
    }

    public class Unadverisement : Operation
    {
        public override string op { get { return "unadvertise"; } } // required
        public string topic; // required

        public Unadverisement(int Id, string Topic) : base(Id)
        {
            topic = Topic;
        }
    }

    public class Publication : Operation
    {
        public override string op { get { return "publish"; } } // required
        public string topic; // required
        public Message msg; // required

        public Publication(int Id, string Topic, Message MessageContents) : base(Id)
        {
            topic = Topic;
            msg = MessageContents;
        }
    }

    public class Subscription : Operation
    {
        public override string op { get { return "subscribe"; } } // required
        public string topic; // required
        public string type; // optional
        public int throttle_rate; // optional
        public int queue_length; // optional
        public int fragment_size; // optional
        public string compression; // optional

        public Subscription(int Id, string Topic, string Type="", int Throttle_rate = 0, int Queue_length = 1, int Fragment_size = int.MaxValue, string Compression = "none") : base(Id)
        {
            topic = Topic;
            type = Type;
            throttle_rate = Throttle_rate;
            queue_length = Queue_length;
            fragment_size = Fragment_size;
            compression = Compression;
        }
        public bool ShouldSerializetype() { return (type != ""); }
        public bool ShouldSerializethrottle_rate() { return (throttle_rate != 0); }
        public bool ShouldSerializequeue_length() { return (queue_length != 1); }
        public bool ShouldSerializefragment_size() { return (fragment_size != int.MaxValue); }
        public bool ShouldSerializecompression() { return (compression != "none"); }

    }

    public class Unsubscription : Operation
    {
        public override string op { get { return "unsubscribe"; } } // required
        public string topic; // required

        public Unsubscription(int Id, string Topic) : base(Id)
        {
            topic = Topic;
        }
    }

    public class ServiceCall : Operation
    {
        public override string op { get { return "call_service"; } } // required
        public string service; // required
        public object args; // optional
        public int fragment_size; // optional
        public string compression; // optional

        public ServiceCall(int Id, string Service, object Args = null, int Fragment_size = int.MaxValue, string Compression = "none") : base(Id)
        {
            service = Service;
            args = Args;
            fragment_size = Fragment_size;
            compression = Compression;
        }
        public bool ShouldSerializeargs() { return (args != null); }
        public bool ShouldSerializefragment_size() { return (fragment_size != int.MaxValue); }
        public bool ShouldSerializecompression() { return (compression != "none"); }
    }

    public class ServiceResponse : Operation
    {
        public override string op { get { return "service_response"; } } // required
        public string service; // required
        public object values; // optional

        public ServiceResponse(int Id, string Service, object Values = null) : base(Id)
        {
            service = Service;
            values = Values;
        }
        public bool ShouldSerializeargs() { return (values != null); }
    }

    /*
    public class ServiceAdverisement : Operation
    {
        public override string op { get { return "advertise_service"; } } // required
        public string type; // required
        public string service; // required


        public ServiceAdverisement(int Id, string Service, string Type) : base(Id)
        {
            service = Service;
            type = Type;
            Id = id;
        }
    }
    public class ServiceUnadverisement : Operation
    {
        public override string op { get { return "unadvertise_service"; } } // required
        public string service; // required

        public ServiceUnadverisement(int Id, string Service) : base(Id)
        {
            service = Service;
        }
    }
    */

}
