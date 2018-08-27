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

using UnityEditor;
using UnityEngine;

namespace RosSharp.Urdf
{
    [SelectionBase]
    public class UrdfCollision : MonoBehaviour
    {
        [SerializeField]
        public UrdfGeometry.GeometryTypes geometryType;

        public static void Create(Transform parent, UrdfGeometry.GeometryTypes type, Transform visualToCopy = null)
        {
            GameObject collisionObject = new GameObject("unnamed");
            collisionObject.transform.SetParentAndAlign(parent);

            UrdfCollision urdfCollision = collisionObject.AddComponent<UrdfCollision>();
            urdfCollision.geometryType = type;

            if (visualToCopy != null)
            {
                if (urdfCollision.geometryType == UrdfGeometry.GeometryTypes.Mesh)
                {
                    GameObject geometryGameObject = UrdfGeometryCollision.CreateMatchingMeshCollision(visualToCopy);
                    geometryGameObject.transform.SetParentAndAlign(collisionObject.transform);
                }
                else
                    UrdfGeometryCollision.Create(collisionObject.transform, type);

                //copy transform values from corresponding UrdfVisual
                collisionObject.transform.position = visualToCopy.position;
                collisionObject.transform.localScale = visualToCopy.localScale;
                collisionObject.transform.rotation = visualToCopy.rotation;
            }
             
            EditorGUIUtility.PingObject(collisionObject);
        }

        public static void Create(Transform parent, Link.Collision collision)
        {
            GameObject collisionObject = new GameObject("unnamed");
            collisionObject.transform.SetParentAndAlign(parent);
            UrdfCollision urdfCollision = collisionObject.AddComponent<UrdfCollision>();
            urdfCollision.geometryType = UrdfGeometry.GetGeometryType(collision.geometry);

            UrdfGeometryCollision.Create(collisionObject.transform, urdfCollision.geometryType, collision.geometry);
            UrdfOrigin.SetTransform(collisionObject.transform, collision.origin);
        }
    
        public Link.Collision GetCollisionData()
        {
            CheckForUrdfCompatibility();

            Link.Geometry geometry = UrdfGeometry.GetGeometryData(geometryType, transform, true);
            string collisionName = gameObject.name == "unnamed" ? null : gameObject.name;

            return new Link.Collision(geometry, collisionName, transform.GetOriginData());
        }

        private void CheckForUrdfCompatibility()
        {
            Transform childTransform = transform.GetChild(0);
            if (childTransform.IsTransformed(geometryType))
                Debug.LogWarning("Changes to the transform of " + childTransform.name + " cannot be exported to URDF. " +
                                 "Make any translation, rotation, or scale changes to the parent Collision object instead.",
                    childTransform);

            if (!transform.HasExactlyOneChild())
                Debug.LogWarning("Only one Geometry element is allowed for each Collision element. In "
                                 + transform.parent.parent.name + ", move each Geometry into its own Visual element.", gameObject);
        }

    }
}