/*
© Siemens AG, 2017-2018
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
        public string id; // optional

        public Operation(string Id = null)
        {
            id = Id;
        }
    }

    public class Adverisement : Operation
    {
        public override string op { get { return "advertise"; } } // required
        public string topic; // required
        public string type; // required

        public Adverisement(string Id, string Topic, string Type) : base(Id)
        {
            topic = Topic;
            type = Type;
        }
    }

    public class Unadverisement : Operation
    {
        public override string op { get { return "unadvertise"; } } // required
        public string topic; // required

        public Unadverisement(string Id, string Topic) : base(Id)
        {
            topic = Topic;
        }
    }

    public class Publication : Operation
    {
        public override string op { get { return "publish"; } } // required
        public string topic; // required
        public Message msg; // required

        public Publication(string Id, string Topic, Message MessageContents) : base(Id)
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

        public Subscription(string Id, string Topic, string Type, int Throttle_rate = 0, int Queue_length = 1, int Fragment_size = int.MaxValue, string Compression = "none") : base(Id)
        {
            topic = Topic;
            type = Type;
            throttle_rate = Throttle_rate;
            queue_length = Queue_length;
            fragment_size = Fragment_size;
            compression = Compression;
        }
    }

    public class Unsubscription : Operation
    {
        public override string op { get { return "unsubscribe"; } } // required
        public string topic; // required

        public Unsubscription(string Id, string Topic) : base(Id)
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

        public ServiceCall(string Id, string Service, object Args = null, int Fragment_size = int.MaxValue, string Compression = "none") : base(Id)
        {
            service = Service;
            args = Args;
            fragment_size = Fragment_size;
            compression = Compression;
        }
    }

    public class ServiceResponse : Operation
    {
        public override string op { get { return "service_response"; } } // required
        public string service; // required
        public object values; // optional
        public bool result;

        public ServiceResponse(string Id, string Service, object Values, bool Result) : base(Id)
        {
            service = Service;
            values = Values;
            result = Result;
        }
    }
    public class ServiceAdvertisement : Operation
    {
        public override string op { get { return "advertise_service"; } } // required
        public string type; // required
        public string service; // required


        public ServiceAdvertisement(string Id, string Service, string Type) : base(Id)
        {
            service = Service;
            type = Type;
            Id = id;
        }
    }
    public class ServiceUnadvertisement : Operation
    {
        public override string op { get { return "unadvertise_service"; } } // required
        public string service; // required

        public ServiceUnadvertisement(string Id, string Service) : base(Id)
        {
            service = Service;
        }
    }
}
