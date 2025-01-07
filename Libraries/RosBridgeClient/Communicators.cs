/*
© Siemens AG, 2017-2019
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

//using RosSharp.RosBridgeClient.Actionlib;

using System;
using System.Text.Json;


namespace RosSharp.RosBridgeClient
{
    public delegate void ServiceResponseHandler<T>(T t) where T : Message;

    public delegate void SubscriptionHandler<T>(T t) where T : Message;

    public delegate bool ServiceCallHandler<Tin, Tout>(Tin tin, out Tout tout) where Tin : Message where Tout : Message;

#region ActionHandlers
#if ROS2

    public delegate void ActionResultResponseHandler<TActionResult>(TActionResult t)
        where TActionResult : Message;

    public delegate void ActionFeedbackResponseHandler<TActionFeedback>(TActionFeedback t)
        where TActionFeedback : Message;

    public delegate void ActionCancelResponseHandler<TActionResult>(TActionResult t)
        where TActionResult : Message;

    public delegate void SendActionGoalHandler<TActionGoal>(TActionGoal t)
        where TActionGoal : Message;

    public delegate void CancelActionGoalHandler(string frameId, string action); // todo: there is no message type for cancel action goal

#endif
#endregion

    internal abstract class Communicator
    {
        public static string GetRosName<T>() where T : Message
        {
            return (string)typeof(T).GetField("RosMessageName").GetRawConstantValue();
        }
    }
    internal abstract class Publisher : Communicator
    {
        internal abstract string Id { get; }
        internal abstract string Topic { get; }

        internal abstract Communication Publish(Message message);

        internal Unadvertisement Unadvertise()
        {
            return new Unadvertisement(Id, Topic);
        }
    }

    internal class Publisher<T> : Publisher where T : Message
    {
        internal override string Id { get; }
        internal override string Topic { get; }

        internal Publisher(string id, string topic, out Advertisement advertisement)
        {
            Id = id;
            Topic = topic;
            advertisement = new Advertisement(Id, Topic, GetRosName<T>());
        }

        internal override Communication Publish(Message message)
        {
            return new Publication<T>(Id, Topic, (T)message);
        }
    }

    internal abstract class Subscriber : Communicator
    {
        internal abstract string Id { get; }
        internal abstract string Topic { get; }
        internal abstract Type TopicType { get; }

        internal abstract void Receive(string message, ISerializer serializer);

        internal Unsubscription Unsubscribe()
        {
            return new Unsubscription(Id, Topic);
        }
    }

    internal class Subscriber<T> : Subscriber where T : Message
    {
        internal override string Id { get; }
        internal override string Topic { get; }
        internal override Type TopicType { get { return typeof(T); } }

        internal SubscriptionHandler<T> SubscriptionHandler { get; }

        private bool _doEnsureThreadSafety = false;
        public bool DoEnsureThreadSafety
        {
            get => _doEnsureThreadSafety;
            set
            {
                _doEnsureThreadSafety= value;
                SetReceiveMethod();
            }
        }

        private readonly object _lock = new object();
        private Action<string, ISerializer> _receiveMethod;

        internal Subscriber(string id, string topic, SubscriptionHandler<T> subscriptionHandler, out Subscription subscription, int throttle_rate = 0, int queue_length = 1, int fragment_size = int.MaxValue, string compression = "none")
        {
            Id = id;
            Topic = topic;
            SubscriptionHandler = subscriptionHandler;
            subscription = new Subscription(id, Topic, GetRosName<T>(), throttle_rate, queue_length, fragment_size, compression);

            SetReceiveMethod();
        }

        private void SetReceiveMethod()
        {
            if (_doEnsureThreadSafety)
            {
                _receiveMethod = ReceiveThreadSafe;
            }
            else
            {
                _receiveMethod = ReceiveNonThreadSafe;
            }
        }

        internal override void Receive(string message, ISerializer serializer)
        {
            _receiveMethod(message, serializer);
        }

        private void ReceiveThreadSafe(string message, ISerializer serializer)
        {
            lock (_lock)
            {
                SubscriptionHandler.Invoke(serializer.Deserialize<T>(message));
            }
        }

        private void ReceiveNonThreadSafe(string message, ISerializer serializer)
        {
            SubscriptionHandler.Invoke(serializer.Deserialize<T>(message));
        }
    }

    internal abstract class ServiceProvider : Communicator
    {
        internal abstract string Service { get; }

        internal abstract Communication Respond(string id, string message, ISerializer serializer);

        internal ServiceUnadvertisement UnadvertiseService()
        {
            return new ServiceUnadvertisement(Service);
        }
    }

    internal class ServiceProvider<Tin, Tout> : ServiceProvider where Tin : Message where Tout : Message
    {
        internal override string Service { get; }
        internal ServiceCallHandler<Tin, Tout> ServiceCallHandler;
        internal ServiceProvider(string service, ServiceCallHandler<Tin, Tout> serviceCallHandler, out ServiceAdvertisement serviceAdvertisement)
        {
            Service = service;
            ServiceCallHandler = serviceCallHandler;
            serviceAdvertisement = new ServiceAdvertisement(service, GetRosName<Tin>());
        }

        internal override Communication Respond(string id, string message, ISerializer serializer)
        {
            bool isSuccess = ServiceCallHandler.Invoke(serializer.Deserialize<Tin>(message), out Tout result);
            return new ServiceResponse<Tout>(id, Service, result, isSuccess);
        }
    }

    internal abstract class ServiceConsumer
    {
        internal abstract string Id { get; }
        internal abstract string Service { get; }
        internal abstract void Consume(string message, ISerializer serializer);
    }

    internal class ServiceConsumer<Tin, Tout> : ServiceConsumer where Tin : Message where Tout : Message
    {
        internal override string Id { get; }
        internal override string Service { get; }
        internal ServiceResponseHandler<Tout> ServiceResponseHandler;

        internal ServiceConsumer(string id,
                                 string service,
                                 ServiceResponseHandler<Tout> serviceResponseHandler,
                                 out Communication serviceCall,
                                 Tin serviceArguments)
        {
            Id = id;
            Service = service;
            ServiceResponseHandler = serviceResponseHandler;
            serviceCall = new ServiceCall<Tin>(id, service, serviceArguments);
        }
        internal override void Consume(string message, ISerializer serializer)
        {
            ServiceResponseHandler.Invoke(serializer.Deserialize<Tout>(message));
        }
    }

#region Action
#if ROS2

    internal abstract class ActionProvider : Communicator
    {
        internal abstract string Action { get; }

        // Feedback and status publication is embedded into "send_goal_response" by ROS Bridge design
        internal abstract Communication RespondResult<TActionResult, TResult>(TActionResult ActionResult) 
            where TActionResult : ActionResult<TResult>
            where TResult : Message;
        internal abstract Communication RespondFeedback<TActionFeedback, TFeedback>(TActionFeedback ActionFeedback)
            where TActionFeedback : ActionFeedback<TFeedback>
            where TFeedback : Message;

        internal abstract void ListenSendGoalAction(string message, ISerializer serializer);
        internal abstract void ListenCancelGoalAction(string frameId, string actionName, ISerializer serializer);

        internal ActionUnadvertisement UnadvertiseAction()
        {
            return new ActionUnadvertisement(Action);
        }
    }

    internal class ActionProvider<TActionGoal> : ActionProvider 
        where TActionGoal : Message

    {
        internal override string Action { get; }

        internal SendActionGoalHandler<TActionGoal> SendActionGoalHandler;
        internal CancelActionGoalHandler CancelActionGoalHandler;

        internal ActionProvider(string action, 
            SendActionGoalHandler<TActionGoal> sendActionGoalHandler,
            CancelActionGoalHandler cancelActionGoalHandler,
            out ActionAdvertisement actionAdvertisement)
        {
            Action = action;
            SendActionGoalHandler = sendActionGoalHandler;
            CancelActionGoalHandler = cancelActionGoalHandler;

            string actionGoalROSName = GetRosName<TActionGoal>();
            actionAdvertisement = new ActionAdvertisement(action, actionGoalROSName.Substring(0, actionGoalROSName.LastIndexOf("ActionGoal"))); 
        }
        
        internal override void ListenSendGoalAction(string message, ISerializer serializer)
        {
            try
            {
                SendActionGoalHandler.Invoke(serializer.Deserialize<TActionGoal>(message));
            }
            catch (Exception ex)
            {
                Console.WriteLine("Error in ListenSendGoalAction: " + ex.Message);
            }
        }

        internal override void ListenCancelGoalAction(string frameId, string actionName, ISerializer serializer)
        {
            try
            {
                CancelActionGoalHandler.Invoke(frameId, actionName);
            }
            catch (Exception ex)
            {
                Console.WriteLine("Error in ListenCancelGoalAction: " + ex.Message);
            }
        }
        internal override Communication RespondFeedback<TActionFeedback, TFeedback>(TActionFeedback actionFeedback)
        {
            return new ActionFeedbackResponse<TFeedback>(
                id: actionFeedback.id,
                action: actionFeedback.action,
                values: actionFeedback.values
            );
        }

        internal override Communication RespondResult<TActionResult, TResult>(TActionResult ActionResult)
        {
            return new ActionResultResponse<TResult>(
                id: ActionResult.id,
                action: ActionResult.action, 
                values: ActionResult.values,
                status: ActionResult.status,
                result: ActionResult.result
            );
        }
    }         

    internal abstract class ActionConsumer : Communicator
    {
        internal abstract string Id { get; }
        internal abstract string Action { get; }
        internal abstract string ActionType { get; }

        internal abstract void ConsumeResultResponse(string incomingJsonResultResponse, ISerializer serializer);
        internal abstract void ConsumeFeedbackResponse(string incomingJsonFeedbackResponse, ISerializer serializer);
        internal abstract void ConsumeCancelResponse(string incomingJsonFeedbackResponse, ISerializer serializer);

        internal abstract Communication SendActionGoalRequest<TActionGoal, TGoal>(TActionGoal actionGoal)
            where TActionGoal : ActionGoal<TGoal>
            where TGoal : Message;

        internal abstract Communication CancelActionGoalRequest(string frameId, string actionName);
    }

    internal class ActionConsumer<TActionResult, TActionFeedback> : ActionConsumer
        where TActionResult : Message
        where TActionFeedback : Message
    {
        internal override string Id { get; }
        internal override string Action { get; }
        internal override string ActionType { get; }

        internal ActionResultResponseHandler<TActionResult> ActionResultResponseHandler;
        internal ActionFeedbackResponseHandler<TActionFeedback> ActionFeedbackResponseHandler;
        internal ActionCancelResponseHandler<TActionResult> ActionCancelResponseHandler;

        internal ActionConsumer(string id,
                                      string action,
                                      ActionResultResponseHandler<TActionResult> actionResultResponseHandler = null,
                                      ActionFeedbackResponseHandler<TActionFeedback> actionFeedbackResponseHandler = null,
                                      ActionCancelResponseHandler<TActionResult> actionCancelResponseHandler = null)
        {
            Id = id;
            Action = action;
            ActionType = GetRosName<TActionResult>().Substring(0, GetRosName<TActionResult>().LastIndexOf("ActionResult"));
            ActionResultResponseHandler = actionResultResponseHandler;
            ActionFeedbackResponseHandler = actionFeedbackResponseHandler;
            ActionCancelResponseHandler = actionCancelResponseHandler;
        }

        internal override void ConsumeResultResponse(string incomingJsonResultResponse, ISerializer serializer)
        {
            try
            {
                ActionResultResponseHandler.Invoke(JsonSerializer.Deserialize<TActionResult>(incomingJsonResultResponse));
            }
            catch (Exception ex)
            {
                Console.WriteLine("Error in ConsumeResultResponse: " + ex.Message);
            }
        }
        internal override void ConsumeFeedbackResponse(string incomingJsonFeedbackResponse, ISerializer serializer)
        {
            try
            {
                ActionFeedbackResponseHandler.Invoke(JsonSerializer.Deserialize<TActionFeedback>(incomingJsonFeedbackResponse));
            }
            catch (Exception ex)
            {
                Console.WriteLine("Error in ConsumeFeedbackResponse: " + ex.Message);
            }
        }
        internal override void ConsumeCancelResponse(string incomingJsonFeedbackResponse, ISerializer serializer)
        {
            try
            {
                ActionCancelResponseHandler.Invoke(serializer.Deserialize<TActionResult>(incomingJsonFeedbackResponse));
            }
            catch (Exception ex)
            {
                Console.WriteLine("Error in ConsumeCancelResponse: " + ex.Message);
            }
        }

        internal override Communication SendActionGoalRequest<TActionGoal, TGoal>(TActionGoal actionGoal)
        {
            return new SendActionGoal<TGoal>(
                id: Id,
                action: actionGoal.action,
                action_type: ActionType,
                args: actionGoal.args,
                feedback: actionGoal.feedback,
                fragment_size: actionGoal.fragment_size,
                compression: actionGoal.compression
            );
        }

        internal override Communication CancelActionGoalRequest(string frameId, string actionName)
        {
            return new CancelActionGoal(Id, frameId, actionName);
        }
    }

#endif
#endregion
}
