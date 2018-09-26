/*
© Siemens AG, 2018
Author: Suzannah Smith (suzannah.smith@siemens.com)

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

using UnityEngine;

namespace RosSharp.Urdf.Editor
{
    public static class UrdfCollisionExtensions
    {
        public static void Create(Transform parent, UrdfRobot.GeometryTypes type, Transform visualToCopy = null)
        {
            GameObject collisionObject = new GameObject("unnamed");
            collisionObject.transform.SetParentAndAlign(parent);

            UrdfCollision urdfCollision = collisionObject.AddComponent<UrdfCollision>();
            urdfCollision.geometryType = type;

            if (visualToCopy != null)
            {
                if (urdfCollision.geometryType == UrdfRobot.GeometryTypes.Mesh)
                    UrdfGeometryCollision.CreateMatchingMeshCollision(collisionObject.transform, visualToCopy);
                else
                    UrdfGeometryCollision.Create(collisionObject.transform, type);

                //copy transform values from corresponding UrdfVisual
                collisionObject.transform.position = visualToCopy.position;
                collisionObject.transform.localScale = visualToCopy.localScale;
                collisionObject.transform.rotation = visualToCopy.rotation;
            }
            else
                UrdfGeometryCollision.Create(collisionObject.transform, type);

            UnityEditor.EditorGUIUtility.PingObject(collisionObject);
        }

        public static void Create(Transform parent, Link.Collision collision)
        {
            GameObject collisionObject = new GameObject("unnamed");
            collisionObject.transform.SetParentAndAlign(parent);
            UrdfCollision urdfCollision = collisionObject.AddComponent<UrdfCollision>();
            urdfCollision.geometryType = UrdfGeometry.GetGeometryType(collision.geometry);

            UrdfGeometryCollision.Create(collisionObject.transform, urdfCollision.geometryType, collision.geometry);
            UrdfOrigin.ImportOriginData(collisionObject.transform, collision.origin);
        }
    
        public static Link.Collision ExportCollisionData(this UrdfCollision urdfCollision)
        {
            urdfCollision.CheckForUrdfCompatibility();

            Link.Geometry geometry = UrdfGeometry.ExportGeometryData(urdfCollision.geometryType, urdfCollision.transform, true);
            string collisionName = urdfCollision.name == "unnamed" ? null : urdfCollision.name;

            return new Link.Collision(geometry, collisionName, UrdfOrigin.ExportOriginData(urdfCollision.transform));
        }

        private static void CheckForUrdfCompatibility(this UrdfCollision urdfCollision)
        {
            Transform childTransform = urdfCollision.transform.GetChild(0);
            if (urdfCollision.IsTransformed())
                Debug.LogWarning("Changes to the transform of " + childTransform.name + " cannot be exported to URDF. " +
                                 "Make any translation, rotation, or scale changes to the parent Collision object instead.",
                    childTransform);

            if (!urdfCollision.transform.HasExactlyOneChild())
                Debug.LogWarning("Only one Geometry element is allowed for each Collision element. In "
                                 + urdfCollision.transform.parent.parent.name + ", move each Geometry into its own Visual element.", urdfCollision.gameObject);
        }

        public static bool IsTransformed(this UrdfCollision urdfCollision)
        {
            Transform childTransform = urdfCollision.transform.GetChild(0);
            //Ignore rotation if geometry is a mesh, because meshes may be rotated during import. 
            return (childTransform.localPosition != Vector3.zero
                    || childTransform.localScale != Vector3.one
                    || (urdfCollision.geometryType != UrdfRobot.GeometryTypes.Mesh && childTransform.localRotation != Quaternion.identity));
        }
    }
}