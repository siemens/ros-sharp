
//#if ROS2
//using RosSharp.RosBridgeClient.MessageTypes.Action;
//using RosSharp.RosBridgeClient.MessageTypes.Std;

//namespace RosSharp.RosBridgeClient
//{
//    public abstract class ActionGoalNew<TGoal> : Message where TGoal : Message
//    {
//        public Header header { get; set; }
//        public GoalInfo goalInfo { get; set; }
//        public TGoal args { get; set; } 

//        // new items
//        public string id { get; set; }
//        public string action { get; set; }
//        public string action_type { get; set; }
//        public bool feedback { get; set; }
//        public int fragment_size { get; set; }
//        public string compression { get; set; }



//        public ActionGoalNew()
//        {
//            header = new Header();
//            goalInfo = new GoalInfo();
//        }

//        public ActionGoalNew(Header header, GoalInfo goalInfo)
//        {
//            this.header = header;
//            this.goalInfo = goalInfo;
//        }
//    }
//}
//#endif