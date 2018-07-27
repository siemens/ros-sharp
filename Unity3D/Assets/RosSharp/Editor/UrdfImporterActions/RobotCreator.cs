using UnityEngine;

namespace RosSharp.UrdfImporter
{
    public static class RobotCreator
    {
        public static GameObject Create(string filename)
        {
            Robot robot = new Robot(filename);
            return robot.Create();
        }
    }
}
