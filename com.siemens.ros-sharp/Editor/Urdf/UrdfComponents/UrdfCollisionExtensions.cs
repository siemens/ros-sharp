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

#if UNITY_EDITOR

using UnityEngine;

namespace RosSharp.Urdf.Editor
{
    public static class UrdfCollisionExtensions
    {
        public static void Create(Transform parent, GeometryTypes type, Transform visualToCopy = null)
        {
            GameObject collisionObject = new GameObject("unnamed");
            collisionObject.transform.SetParentAndAlign(parent);

            UrdfCollision urdfCollision = collisionObject.AddComponent<UrdfCollision>();
            urdfCollision.GeometryType = type;

            if (visualToCopy != null)
            {
                if (urdfCollision.GeometryType == GeometryTypes.Mesh)
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
            urdfCollision.GeometryType = UrdfGeometry.GetGeometryType(collision.geometry);

            UrdfGeometryCollision.Create(collisionObject.transform, urdfCollision.GeometryType, collision.geometry);
            UrdfOrigin.ImportOriginData(collisionObject.transform, collision.origin);
        }
    
        public static Link.Collision ExportCollisionData(this UrdfCollision urdfCollision)
        {
            UrdfGeometry.CheckForUrdfCompatibility(urdfCollision.transform, urdfCollision.GeometryType);

            Link.Geometry geometry = UrdfGeometry.ExportGeometryData(urdfCollision.GeometryType, urdfCollision.transform, true);
            string collisionName = urdfCollision.name == "unnamed" ? null : urdfCollision.name;

            return new Link.Collision(geometry, collisionName, UrdfOrigin.ExportOriginData(urdfCollision.transform));
        }
    }
}

#endif