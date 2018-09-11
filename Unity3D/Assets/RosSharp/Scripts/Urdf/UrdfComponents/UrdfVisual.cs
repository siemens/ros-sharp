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
    public class UrdfVisual : MonoBehaviour
    {
        [SerializeField]
        public UrdfGeometry.GeometryTypes geometryType;

        public static void Create(Transform parent, UrdfGeometry.GeometryTypes type)
        {
            GameObject visualObject = new GameObject("unnamed");
            visualObject.transform.SetParentAndAlign(parent);
            UrdfVisual urdfVisual = visualObject.AddComponent<UrdfVisual>();

            urdfVisual.geometryType = type;
            GameObject geometryGameObject = UrdfGeometryVisual.Create(visualObject.transform, type);

            if (geometryGameObject == null) return;
            
            EditorGUIUtility.PingObject(visualObject);
        }

        public static void Create(Transform parent, Link.Visual visual)
        {
            GameObject visualObject = new GameObject(visual.name ?? "unnamed");
            visualObject.transform.SetParentAndAlign(parent);
            UrdfVisual urdfVisual = visualObject.AddComponent<UrdfVisual>();
            urdfVisual.geometryType = UrdfGeometry.GetGeometryType(visual.geometry);
            
            UrdfGeometryVisual.Create(visualObject.transform, urdfVisual.geometryType, visual.geometry);
            UrdfMaterialHandler.SetUrdfMaterial(visualObject, visual.material);
            UrdfOrigin.SetTransformFromUrdf(visualObject.transform, visual.origin);
        }


        public void AddCorrespondingCollision()
        {
            UrdfCollisions collisions = GetComponentInParent<UrdfLink>().GetComponentInChildren<UrdfCollisions>();
            UrdfCollision.Create(collisions.transform, geometryType, transform);
        }

        public Link.Visual GetVisualData()
        {
            CheckForUrdfCompatibility();

            Link.Geometry geometry = UrdfGeometry.GetGeometryData(geometryType, transform);

            Link.Visual.Material material = UrdfMaterial.GetMaterialData(gameObject.GetComponentInChildren<MeshRenderer>().sharedMaterial);
            string visualName = gameObject.name == "unnamed" ? null : gameObject.name;

            return new Link.Visual(geometry, visualName, UrdfOrigin.ExportOriginToUrdf(transform), material);
        }
        
        private void CheckForUrdfCompatibility()
        {
            Transform childTransform = transform.GetChild(0);
            if (IsTransformed())
                Debug.LogWarning("Changes to the transform of " + childTransform.name + " cannot be exported to URDF. " +
                                 "Make any translation, rotation, or scale changes to the parent Visual object instead.",
                                  childTransform);

            if (!transform.HasExactlyOneChild()) 
                Debug.LogWarning("Only one Geometry element is allowed for each Visual element. In "
                                 + transform.parent.parent.name + ", move each Geometry into its own Visual element.", gameObject);
        }
        
        public bool IsTransformed()
        {
            Transform childTransform = transform.GetChild(0);

            //Ignore rotation if geometry is a mesh, because meshes may be rotated during import. 
            return (childTransform.localPosition != Vector3.zero
                    || childTransform.localScale != Vector3.one
                    || (geometryType != UrdfGeometry.GeometryTypes.Mesh && childTransform.localRotation != Quaternion.identity));
        }
    }
}