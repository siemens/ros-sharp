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
        public UrdfGeometry.GeometryTypes geometryType;

        public void Initialize(UrdfGeometry.GeometryTypes type, Transform visualToCopy = null)
        {
            geometryType = type;

            GameObject geometryGameObject = null;
            
            if (visualToCopy != null && geometryType == UrdfGeometry.GeometryTypes.Mesh) 
            {
                //Generate copy of visual object, add MeshColliders, and remove MeshRenderers.
                geometryGameObject = Instantiate(visualToCopy.GetChild(0).gameObject);
                geometryGameObject.name = visualToCopy.GetChild(0).name;

                MeshFilter[] meshFilters = geometryGameObject.GetComponentsInChildren<MeshFilter>();
                foreach (MeshFilter meshFilter in meshFilters)
                {
                    GameObject child = meshFilter.gameObject;
                    MeshCollider meshCollider = child.AddComponent<MeshCollider>();
                    meshCollider.sharedMesh = meshFilter.sharedMesh;
                    DestroyImmediate(child.GetComponent<MeshRenderer>());
                    DestroyImmediate(meshFilter);
                }
            }
            else
                geometryGameObject = UrdfGeometry.GenerateCollisionGeometry(type);

            geometryGameObject.transform.SetParentAndAlign(gameObject.transform);

            //copy transform values from corresponding UrdfVisual 
            if (visualToCopy != null)
            { 
                transform.localPosition = visualToCopy.localPosition;
                transform.localScale = visualToCopy.localScale;
                transform.localEulerAngles = visualToCopy.localEulerAngles;  
            }
            
            EditorGUIUtility.PingObject(gameObject);
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