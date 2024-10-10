using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class CustomJoyAxisReader : MonoBehaviour, IAxisReader
    {
        private Joystick joystick;
        
        private void Start()
        {
            joystick = GetComponent<Joystick>();
        }

        [SerializeField]
        private AxisType axisType;


        public string Name
        {
            get
            {
                return axisType == AxisType.Horizontal ? joystick.HorizontalAxisName : joystick.VerticalAxisName;
            }
        }

        public float Read()
        {
            return axisType == AxisType.Horizontal ? joystick.Horizontal() : joystick.Vertical();
        }

        private enum AxisType
        {
            Horizontal,
            Vertical
        }
    }
}
