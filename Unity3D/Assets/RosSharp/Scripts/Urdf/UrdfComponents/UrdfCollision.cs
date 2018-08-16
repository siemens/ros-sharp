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

namespace RosSharp.Urdf.Export
{
    [SelectionBase]
    public class UrdfCollision : MonoBehaviour
    {
        [SerializeField]
        private UrdfVisuals.GeometryTypes geometryType;

        public void Initialize(UrdfVisuals.GeometryTypes type)
        {
            geometryType = type;

            GameObject geometryGameObject = new GameObject(type.ToString());
            geometryGameObject.transform.SetParentAndAlign(gameObject.transform);

            switch (type)
            {
                case UrdfVisuals.GeometryTypes.Box:
                    geometryGameObject.AddComponent<BoxCollider>();
                    break;
                case UrdfVisuals.GeometryTypes.Cylinder:
                    geometryGameObject.AddComponent<CapsuleCollider>();
                    break;
                case UrdfVisuals.GeometryTypes.Sphere:
                    geometryGameObject.AddComponent<SphereCollider>();
                    break;
                case UrdfVisuals.GeometryTypes.Mesh:
                    geometryGameObject.AddComponent<MeshCollider>();
                    break;
            }

            geometryGameObject.hideFlags ^= HideFlags.NotEditable;
            EditorGUIUtility.PingObject(geometryGameObject);
        }


        //TODO: Move duplicated Geometry code to UrdfGeometry class
        public Link.Collision GetCollisionData()
        {
            Link.Geometry geometry = null;
            //TODO: Get rid of default values, actually get info from the real meshes
            switch (geometryType)
            {
                //TODO: Get size of collider 
                case UrdfVisuals.GeometryTypes.Box:
                    geometry = new Link.Geometry(new Link.Geometry.Box(transform.GetUrdfSize()));
                    break;
                case UrdfVisuals.GeometryTypes.Cylinder:
                    geometry = new Link.Geometry(null, new Link.Geometry.Cylinder(transform.GetRadius(), transform.GetCylinderHeight()));
                    break;
                case UrdfVisuals.GeometryTypes.Sphere:
                    geometry = new Link.Geometry(null, null, new Link.Geometry.Sphere(transform.GetRadius()));
                    break;
                case UrdfVisuals.GeometryTypes.Mesh:
                    geometry = new Link.Geometry(null, null, null, new Link.Geometry.Mesh("test.dae", null));
                    break;
            }

            //TODO: Fill in optional values, like origin, etc

            return new Link.Collision(geometry, null, transform.GetOriginData());
        }
    }
}