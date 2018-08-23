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
        private UrdfGeometry.GeometryTypes geometryType;

        public void Initialize(UrdfGeometry.GeometryTypes type, Transform visualTransform = null)
        {
            geometryType = type;

            GameObject geometryGameObject = null;
            
            if (visualTransform != null && geometryType == UrdfGeometry.GeometryTypes.Mesh) 
            {
                //Generate copy of visual object, add MeshColliders, and remove MeshRenderers.
                geometryGameObject = Instantiate(visualTransform.GetChild(0).gameObject);
                geometryGameObject.name = visualTransform.GetChild(0).name;

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
            if (visualTransform != null)
            { 
                transform.localPosition = visualTransform.localPosition;
                transform.localScale = visualTransform.localScale;
                transform.localEulerAngles = visualTransform.localEulerAngles;  
            }
            
            EditorGUIUtility.PingObject(gameObject);
        }
    
        //TODO: Move duplicated Geometry code to UrdfGeometry class
        public Link.Collision GetCollisionData()
        {
            Link.Geometry geometry = UrdfGeometry.GetGeometryData(geometryType, transform, true);
            string collisionName = gameObject.name == "unnamed" ? null : gameObject.name;

            return new Link.Collision(geometry, collisionName, transform.GetOriginData());
        }
    }
}