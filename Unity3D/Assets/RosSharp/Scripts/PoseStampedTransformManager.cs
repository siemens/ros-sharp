using UnityEngine;

namespace RosSharp
{
    public class PoseStampedTransformManager : MonoBehaviour
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