using System;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.ActionTutorialsInterfaces;
using RosSharp.RosBridgeClient.Actionlib;


namespace RosBridgeClientTest
{
    public class ActionTest
    {
        static readonly string uri = "ws://localhost:9090";
        public static RosSocket rosSocket = new RosSocket(new RosSharp.RosBridgeClient.Protocols.WebSocketNetProtocol(uri));

        private static FibonacciActionServer fibonacciActionServer;

        public static void Main(string[] args)
        {
#if ROS2
            /// --- Server Test ---

            //fibonacciActionServer = new FibonacciActionServer("/fibonacci", rosSocket, new Log(x => Console.WriteLine(x)));
            //fibonacciActionServer.Initialize();

            //Console.WriteLine("Waiting for goal. Press any key to stop server.");
            //Console.ReadKey(true);

            //fibonacciActionServer.Terminate();
            //rosSocket.Close();




            /// --- Client Test ---

            FibonacciActionClient fibonacciActionClient = new FibonacciActionClient("/fibonacci", rosSocket);

            fibonacciActionClient.SetActionGoal(new FibonacciGoal
            {
                order = 10
            });

            Console.WriteLine("Order is: " + fibonacciActionClient.GetActionGoal().args.order);

            fibonacciActionClient.SendGoal();

            Console.WriteLine("Press any key to cancel."); // Cancel the action if not finished
            Console.ReadKey(true);

            if (!fibonacciActionClient.lastResultSuccess)
            {
                Console.WriteLine("Action not completed. Cancelling...");
                fibonacciActionClient.CancelGoal();
                Console.ReadKey(true);
            }
            else
            {
                Console.WriteLine("Action completed.");
            }
#endif
        }

    }


}
