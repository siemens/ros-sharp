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

using System.IO;
using UnityEditor;
using UnityEngine;

namespace RosSharp.Urdf.Export
{
    [SelectionBase]
    public class UrdfVisual : MonoBehaviour
    {
        [SerializeField]
        private UrdfGeometry.GeometryTypes geometryType;

        public void Initialize(UrdfGeometry.GeometryTypes type)
        {
            geometryType = type;
            GameObject geometryGameObject = UrdfGeometry.GenerateVisualGeometry(type);

            if (geometryGameObject == null) return;

            geometryGameObject.transform.SetParentAndAlign(gameObject.transform);
            transform.DestroyImmediateIfExists<Collider>();

            EditorGUIUtility.PingObject(gameObject);
        }

        public void AddCorrespondingCollision()
        {
            GetComponentInParent<UrdfLink>().GetComponentInChildren<UrdfCollisions>()
                .AddColision(geometryType, transform);
        }

        public Link.Visual GetVisualData()
        {
            CheckForUrdfCompatibility();

            Link.Geometry geometry = UrdfGeometry.GetGeometryData(geometryType, transform);

            Link.Visual.Material material = UrdfMaterial.GetMaterialData(gameObject.GetComponentInChildren<MeshRenderer>().sharedMaterial);
            string visualName = gameObject.name == "unnamed" ? null : gameObject.name;

            return new Link.Visual(geometry, visualName, transform.GetOriginData(), material);
        }
        
        private void CheckForUrdfCompatibility()
        {
            Transform childTransform = transform.GetChild(0);
            if (childTransform != null &&
                (childTransform.localPosition != Vector3.zero || childTransform.localScale != Vector3.one))
                Debug.LogWarning("Changes to the transform of " + childTransform.name + " cannot be exported to URDF. " +
                                 "Make any translation, rotation, or scale changes to the parent Visual object instead.",
                                  childTransform);

            if (transform.childCount > 1)
                Debug.LogWarning("Only one Geometry element is allowed for each Visual element. One the first one will be exported to Urdf. In link "
                                 + transform.parent.parent.name + ", move each Geometry into its own Visual element.", gameObject);
        }
    }
}