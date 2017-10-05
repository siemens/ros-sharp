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


using System;
using System.Linq;
using System.Collections.Generic;
using WebSocketSharp;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using System.Text;

namespace RosBridgeClient
{
    public class RosSocket
    {
        #region Public
        public RosSocket(string url)
        {
            webSocket = new WebSocket(url);
            webSocket.OnMessage += (sender, e) => recievedOperation((WebSocket)sender, e);
            webSocket.Connect();
        }

        public void Close()
        {
            while (publishers.Count > 0)
                Unadvertize(publishers.First().Key);

            while (subscribers.Count > 0)
                Unsubscribe(subscribers.First().Key);

            webSocket.Close();
        }

        public delegate void ServiceHandler(object obj);
        public delegate void MessageHandler(Message message);

        public int Advertize(string topic, string type)
        {
            int id = generateId();
            publishers.Add(id, new Publisher(topic));

            sendOperation(new Adverisement(id, topic, type));
            return id;
        }

        public void Publish(int id, Message msg)
        {
            Publisher publisher;
            if (publishers.TryGetValue(id, out publisher))
                sendOperation(new Publication(id, publisher.Topic, msg));               
        }

        public void Unadvertize(int id)
        {
            sendOperation(new Unadverisement(id, publishers[id].Topic));
            publishers.Remove(id);
        }

        public int Subscribe(string topic, string type, MessageHandler messageHandler, int throttle_rate = 0, int queue_length = 1, int fragment_size = int.MaxValue, string compression = "none")
        {
            int id = generateId();
            subscribers.Add(id, new Subscriber(topic, type, messageHandler));

            sendOperation(new Subscription(id, topic, type, throttle_rate, queue_length, fragment_size, compression));
            
            return id;
        }

        public void Unsubscribe(int id)
        {
            sendOperation(new Unsubscription(id, subscribers[id].topic));
            subscribers.Remove(id);
        }

        public int CallService(string service, Type objectType, ServiceHandler serviceHandler,  object args = null)
        {
            int id = generateId();
            serviceCallers.Add(id, new ServiceCaller(service, objectType, serviceHandler));

            sendOperation(new ServiceCall(id, service, args));
            return id;
        }
        #endregion

        #region Private

        internal struct Publisher
        {
            internal string Topic;
            internal Publisher(string topic)
            {
                Topic = topic;
            }
        }

        internal struct Subscriber
        {
            internal string topic;
            internal string type;
            internal MessageHandler messageHandler;
            internal Subscriber(string Topic, string Type, MessageHandler MessageHandler)
            {
                topic = Topic;
                type = Type;
                messageHandler = MessageHandler;
            }
        }
        internal struct ServiceCaller
        {
            internal string service;
            internal Type objectType;
            internal ServiceHandler serviceHandler;
            internal ServiceCaller(string Service, Type ObjectType, ServiceHandler ServiceHandler)
            {
                service = Service;
                objectType = ObjectType;
                serviceHandler = ServiceHandler;
            }
        }

        private WebSocket webSocket;
        private Dictionary<int, Publisher> publishers = new Dictionary<int, Publisher>();
        private Dictionary<int, Subscriber> subscribers = new Dictionary<int, Subscriber>();
        private Dictionary<int, ServiceCaller> serviceCallers = new Dictionary<int, ServiceCaller>();

        private Dictionary<string, Type> messageObjects = new Dictionary<string, Type>
        {
            { "geometry_msgs/Twist", typeof(GeometryTwist) },
            { "std_msgs/String", typeof(StandardString) },
            { "sensor_msgs/JointState", typeof(SensorJointStates) },
            { "geometry_msgs/Vector3", typeof(GeometryVector3) },
            { "nav_msgs/Odometry", typeof(NavigationOdometry) },
            { "std_msgs/Header", typeof(StandardHeader) },
            { "geometry_msgs/PoseWithCovariance", typeof(GeometryPoseWithCovariance)},
            { "geometry_msgs/TwistWithCovariance", typeof(GeometryTwistWithCovariance)},
            { "geometry_msgs/Pose", typeof(GeometryPose)},
            { "geometry_msgs/Point", typeof(GeometryPoint)},
            { "geometry_msgs/Quaternion", typeof(GeometryQuaternion) },
            { "sensor_msgs/PointCloud2", typeof(SensorPointCloud2) },
            { "sensor_msgs/PointField" , typeof(SensorPointField)},
            { "sensor_msgs/Image", typeof(SensorImage) },
            { "sensor_msgs/CompressedImage",typeof(SensorCompressedImage)},
            {"nav_msgs/MapMetaData", typeof(NavigationMapMetaData) },
            {"nav_msgs/OccupancyGrid", typeof(NavigationOccupancyGrid) },
        };

        private void recievedOperation(object sender, MessageEventArgs e)
        {
            JObject operation = Deserialize(e.RawData);

#if DEBUG
            Console.WriteLine("Recieved " + operation.GetOperation());
            Console.WriteLine(JsonConvert.SerializeObject(operation, Formatting.Indented));
#endif

            switch (operation.GetOperation())
            {
                case "publish":
                    {
                        recievedPublish(operation, e.RawData);
                        return;
                    }
                case "service_response":
                    {
                        recievedServiceResponse(operation, e.RawData);
                        return;
                    }
            }
        }

        private void recievedServiceResponse(JObject serviceResponse, byte[] rawData)
        {
            ServiceCaller serviceCaller;
            bool foundById = serviceCallers.TryGetValue(serviceResponse.GetServiceId(), out serviceCaller);

            if (!foundById)
                serviceCaller = serviceCallers.Values.FirstOrDefault(x => x.service.Equals(serviceResponse.GetService()));

           
            JObject jObject = (JObject) serviceResponse.GetValues();
            Type type = serviceCaller.objectType;
            if (type != null)
                serviceCaller.serviceHandler?.Invoke(jObject.ToObject(type));
            else
                serviceCaller.serviceHandler?.Invoke(jObject);
        }
        
        private void recievedPublish(JObject publication, byte[] rawData)
        {
            Subscriber subscriber;
            bool foundById = subscribers.TryGetValue(publication.GetServiceId(), out subscriber);

            if (!foundById)
                subscriber = subscribers.Values.FirstOrDefault(x => x.topic.Equals(publication.GetTopic()));

            subscriber.messageHandler?.Invoke((Message)publication.GetMessage().ToObject(messageObjects[subscriber.type]));
        }

        private void sendOperation(Operation operation)
        {
#if DEBUG
            Console.WriteLine(JsonConvert.SerializeObject(operation, Formatting.Indented));
#endif
            webSocket.SendAsync(Serialize(operation), null);
        }
        public static byte[] Serialize(object obj)
        {
            string json = JsonConvert.SerializeObject(obj);
            byte[] buffer = Encoding.ASCII.GetBytes(json);
            int I = json.Length;
            int J = buffer.Length;
            return buffer;
        }

        public static JObject Deserialize(byte[] buffer)
        {
            string ascii = Encoding.ASCII.GetString(buffer, 0, buffer.Length);
            int I = ascii.Length;
            int J = buffer.Length;
            return JsonConvert.DeserializeObject<JObject>(ascii);
        }

        private static int generateId()
        {
            return Guid.NewGuid().GetHashCode();
        }
        #endregion       
    }
}
