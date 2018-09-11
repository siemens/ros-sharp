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

using System;
using RosSharp.Urdf;
using UnityEngine;
using Object = UnityEngine.Object;

namespace RosSharp
{
    public static class TransformExtensions
    {
        private const int RoundDigits = 6;

        public static void DestroyImmediateIfExists<T>(this Transform transform) where T : Component
        {
            T component = transform.GetComponent<T>();
            if (component != null)
                Object.DestroyImmediate(component);
        }

        public static T AddComponentIfNotExists<T>(this Transform transform) where T : Component
        {
            T component = transform.GetComponent<T>();
            if (component == null)
                component = transform.gameObject.AddComponent<T>();
            return component;
        }

        public static void SetParentAndAlign(this Transform transform, Transform parent, bool keepLocalTransform = true)
        {
            Vector3 localPosition = transform.localPosition;
            Quaternion localRotation = transform.localRotation;
            transform.parent = parent;
            if (keepLocalTransform)
            {
                transform.position = transform.parent.position + localPosition;
                transform.rotation = transform.parent.rotation * localRotation;
            }
            else
            {
                transform.localPosition = Vector3.zero;
                transform.localRotation = Quaternion.identity;
            }
        }

        public static bool HasExactlyOneChild(this Transform transform)
        {
            return transform.childCount == 1;
        }

        public static void MoveChildTransformToParent(this Transform parent, bool transferRotation = true)
        {
            //Detach child in order to get a transform indenpendent from parent
            Transform childTransform = parent.GetChild(0);
            parent.DetachChildren();

            //Copy transform from child to parent
            parent.position = childTransform.position;
            parent.localScale = childTransform.localScale;

            if (transferRotation)
            {
                parent.rotation = childTransform.rotation;
                childTransform.localRotation = Quaternion.identity;
            }

            childTransform.parent = parent;

            childTransform.localPosition = Vector3.zero;
            childTransform.localScale = Vector3.one;
        }

        public static Vector3 Ros2Unity(this Vector3 vector3)
        {
            return new Vector3(-vector3.y, vector3.z, vector3.x);
        }

        public static Vector3 Unity2Ros(this Vector3 vector3)
        {
            return new Vector3(vector3.z, -vector3.x, vector3.y);
        }

        public static Vector3 Ros2UnityScale(this Vector3 vector3)
        {
            return new Vector3(vector3.y, vector3.z, vector3.x);
        }

        public static Vector3 Unity2RosScale(this Vector3 vector3)
        {
            return new Vector3(vector3.z, vector3.x, vector3.y);
        }

        public static Quaternion Ros2Unity(this Quaternion quaternion)
        {
            return new Quaternion(-quaternion.x, -quaternion.z, -quaternion.y, quaternion.w);
        }

        public static Quaternion Unity2Ros(this Quaternion quaternion)
        {
            return new Quaternion(-quaternion.x, -quaternion.z, -quaternion.y, quaternion.w);
        }

        public static double[] ToRoundedDoubleArray(this Vector3 vector3)
        {
            double[] arr = new double[3];
            for (int i = 0; i < 3; i++)
                arr[i] = Math.Round(vector3[i], RoundDigits);

            return arr;
        }

        public static Vector3 ToVector3(this double[] array)
        {
            return new Vector3((float)array[0], (float)array[1], (float)array[2]);
        }
    }
}