﻿/*
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
using RosSharp.Urdf.Export;
using UnityEngine;
using Object = UnityEngine.Object;

namespace RosSharp
{
    public static class TransformExtensions
    {
        private const int RoundDigits = 4;

        public static void DestroyImmediateIfExists<T>(this Transform transform) where T : Component
        {
            T component = transform.GetComponent<T>();
            if (component != null)
                Object.DestroyImmediate(component);
        }

        public static void DestroyChildrenImmediate(this Transform transform)
        {
            while(transform.childCount != 0)
                Object.DestroyImmediate(transform.GetChild(0).gameObject);
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

        public static bool IsTransformed(this Transform transform, UrdfGeometry.GeometryTypes geometryType)
        {
            //Ignore rotation if geometry is a mesh. This is because meshes may be rotated during import. 
            return (transform.localPosition != Vector3.zero
                    || transform.localScale != Vector3.one
                    || (geometryType != UrdfGeometry.GeometryTypes.Mesh && transform.localRotation != Quaternion.identity));
        }

        public static  void MoveChildTransformToParent(this Transform parent)
        {
            //Detach child in order to get a transform indenpendent from parent
            Transform childTransform = parent.GetChild(0);
            parent.DetachChildren();

            //Copy transform from child to parent
            parent.position = childTransform.position;
            parent.rotation = childTransform.rotation;
            parent.localScale = childTransform.localScale;

            //Re-attach parent and reset child transform
            childTransform.SetParentAndAlign(parent, false);
        }

        public static Origin GetOriginData(this Transform transform)
        {
            double[] xyz = transform.GetUrdfXyz();
            double[] rpy = transform.GetUrdfRpy();

            if (xyz != null || rpy != null)
                return new Origin(xyz, rpy);

            return null;
        }

        private static double[] GetUrdfXyz(this Transform transform)
        {
            Vector3 xyzVector = transform.localPosition.Unity2Ros();
            return xyzVector == Vector3.zero ? null: xyzVector.ToRoundedDoubleArray();
        }

        private static double[] GetUrdfRpy(this Transform transform)
        {
            Vector3 rpyVector = new Vector3(
                -transform.localEulerAngles.z * Mathf.Deg2Rad,
                transform.localEulerAngles.x * Mathf.Deg2Rad,
                -transform.localEulerAngles.y * Mathf.Deg2Rad);

            return rpyVector == Vector3.zero ? null : rpyVector.ToRoundedDoubleArray();
        }

        public static double[] GetUrdfSize(this Transform transform)
        {
            return new double []
            {
                Math.Round(transform.localScale.z, RoundDigits),
                Math.Round(transform.localScale.x, RoundDigits),
                Math.Round(transform.localScale.y, RoundDigits)
            };
        }

        public static double GetRadius(this Transform transform)
        {
            return Math.Round(transform.localScale.x / 2, RoundDigits);
        }

        public static double GetCylinderHeight(this Transform transform)
        {
            return Math.Round(transform.localScale.y * 2, RoundDigits);
        }
        
        public static Transform FindDeepChild(this Transform parent, string name)
        {
            Transform result = parent.Find(name);
            if (result != null)
                return result;
            foreach (Transform child in parent)
            {
                result = child.FindDeepChild(name);
                if (result != null)
                    return result;
            }
            return null;
        }

        public static Vector3 Ros2Unity(this Vector3 vector3)
        {
            return new Vector3(-vector3.y, vector3.z, vector3.x);
        }

        public static Vector3 Unity2Ros(this Vector3 vector3)
        {
            return new Vector3(vector3.z, -vector3.x, vector3.y);
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
    }
}