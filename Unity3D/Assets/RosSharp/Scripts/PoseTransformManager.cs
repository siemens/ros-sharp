
using UnityEngine;

namespace RosSharp
{
    public class PoseTransformManager : MonoBehaviour
    {
        private Vector3 position;
        private Quaternion rotation;
        private bool doUpdate;

        private void Update()
        {
            if (doUpdate)
            {
                transform.position = position;
                transform.rotation = rotation;
                doUpdate = false;
                transform.hasChanged = false;  // to avoid republishing in a infinite loop
                return;
            }
        }

        public void updateTransform(Vector3 _position, Quaternion _rotation)
        {
            position = _position;
            rotation = _rotation;
            doUpdate = true;
        }
    }
}