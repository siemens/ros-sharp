using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Std;
using System;
using Windows.Foundation;
using Windows.UI.Core;

// This example code shows how you could implement the required main function for a 
// Console UWP Application. You can replace all the code inside Main with your own custom code.

// You should also change the Alias value in the AppExecutionAlias Extension in the 
// Package.appxmanifest to a value that you define. To edit this file manually, right-click
// it in Solution Explorer and select View Code, or open it with the XML Editor.

namespace RosBridgeUWPTestConsole
{
    class Program
    {
        static string LastMsg = "None";
        static RosSocket rosSocket;
        static void Main(string[] args)
        {
            Console.WriteLine("test");
            //IAsyncAction asyncAction = Windows.System.Threading.ThreadPool.RunAsync(
            //(workItem) =>
            //{
            //    DoWork();
            //});
           DoWork();

        //while(Console.ReadLine() != "e")
        //{
        //    Console.WriteLine(LastMsg);
        //}


            Console.ReadLine();
        }

        public static void WriteToConsole(string msg)
        {
            //Console.WriteLine(msg);
            System.IO.File.AppendAllText(@"C:\Users\ericv\Desktop\test.txt", "\n" + msg);
        }

        private static void OnClosed(object sender, EventArgs e)
        {
            WriteToConsole("Closed");
        }

        private static void OnConnected(object sender, EventArgs e)
        {
            WriteToConsole("Yes!");
        }

        private static void DoWork()
        {
            string uri = "ws://localhost:9090";
            WriteToConsole($"Trying to connect to RosBridge via {uri}");
            RosSharp.RosBridgeClient.Protocols.WebSocketNetProtocol webSocketNetProtocol = new RosSharp.RosBridgeClient.Protocols.WebSocketNetProtocol(uri);
            return;
            var protocol = RosSharp.RosBridgeClient.Protocols.ProtocolInitializer.GetProtocol(RosSharp.RosBridgeClient.Protocols.Protocol.WebSocketNET, uri);
            protocol.OnConnected += OnConnected;
            protocol.OnClosed += OnClosed;
            rosSocket = new RosSocket(protocol, RosSocket.SerializerEnum.Newtonsoft_JSON);
            rosSocket.Subscribe<Clock>("/clock", ReceiveMessage);
            //rosSocket.Subscribe<RosSharp.RosBridgeClient.MessageTypes.Std.String>("/clock", ReceiveMessage);
        }

        private static void ReceiveMessage(Clock msg)
        {
            LastMsg = "secs " + msg.clock.secs;
            WriteToConsole(LastMsg);
        }


    }


    public class Clock : Message
    {
        public override string RosMessageName => "rosgraph_msgs/Clock";

        public Time clock;
    }
}