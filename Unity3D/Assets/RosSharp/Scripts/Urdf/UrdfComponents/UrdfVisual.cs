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

using System;
using UnityEditor;
using UnityEngine;

namespace RosSharp.Urdf.Export
{
    public class UrdfVisual : MonoBehaviour
    {
        private const int RoundDigits = 4;

        public UrdfVisuals.GeometryTypes geometryType;

        public void Initialize(UrdfVisuals.GeometryTypes type)
        {
            geometryType = type;

            GameObject geometryGameObject = null;

            switch (type)
            {
                case UrdfVisuals.GeometryTypes.Box:
                    geometryGameObject = GameObject.CreatePrimitive(PrimitiveType.Cube);
                    break;
                case UrdfVisuals.GeometryTypes.Cylinder:
                    geometryGameObject = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                    break;
                case UrdfVisuals.GeometryTypes.Sphere:
                    geometryGameObject = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    break;
                case UrdfVisuals.GeometryTypes.Mesh:
                    geometryGameObject = new GameObject();
                    geometryGameObject.AddComponent<MeshFilter>();
                    geometryGameObject.AddComponent<MeshRenderer>();
                    break;
            }

            geometryGameObject.transform.SetParentAndAlign(gameObject.transform);
            DestroyImmediate(geometryGameObject.GetComponent<Collider>());

            EditorGUIUtility.PingObject(geometryGameObject);
        }

        public Link.Visual GetVisualData()
        {
            Link.Geometry geometry = null;
            //TODO: Get rid of default values, actually get info from the real meshes
            switch (geometryType)
            {
                case UrdfVisuals.GeometryTypes.Box:
                    geometry = new Link.Geometry(new Link.Geometry.Box(transform.GetUrdfSize()));
                    break;
                case UrdfVisuals.GeometryTypes.Cylinder:
                    geometry = new Link.Geometry(
                        null, 
                        new Link.Geometry.Cylinder(transform.GetRadius(), transform.GetCylinderHeight())
                    );
                    break;
                case UrdfVisuals.GeometryTypes.Sphere:
                    geometry = new Link.Geometry(null, null, new Link.Geometry.Sphere(transform.GetRadius()));
                    break;
                case UrdfVisuals.GeometryTypes.Mesh:
                    //TODO: figure out how to export meshes (both .stl and .dae)
                    //TODO: move code for finding asset path to assetpathhandler
                    MeshFilter filter = gameObject.GetComponentInChildren<MeshFilter>();
                    string path = AssetDatabase.GetAssetPath(PrefabUtility.GetCorrespondingObjectFromSource(filter.gameObject));
                    if (path.Contains(".dae"))
                        path = path.Substring(Application.dataPath.Length);
                    geometry = new Link.Geometry(null, null, null, new Link.Geometry.Mesh(path, transform.GetUrdfSize()));
                    break;
            }

            //TODO: Fill in more optional values, like name
            Link.Visual.Material material = GetMaterialData(gameObject.GetComponentInChildren<MeshRenderer>().sharedMaterial);

            return new Link.Visual(geometry, null, transform.GetOriginData(), material);
        }

        private Link.Visual.Material GetMaterialData(Material material)
        {
            if (!material.color.Equals(Color.clear))
                return new Link.Visual.Material(material.name, new Link.Visual.Material.Color(GetRgba(material)));

            if (material.mainTexture != null)
            {
                // TODO test if textures work
                Debug.Log(AssetDatabase.GetAssetPath(material.mainTexture));
                Link.Visual.Material.Texture texture = new Link.Visual.Material.Texture(AssetDatabase.GetAssetPath(material.mainTexture));
                return new Link.Visual.Material(material.name, null, texture);
            }

            return null;
        }

        private static double[] GetRgba(Material material)
        {
            return new double[]
            {
                Math.Round(material.color.r, RoundDigits),
                Math.Round(material.color.g, RoundDigits),
                Math.Round(material.color.b, RoundDigits),
                Math.Round(material.color.a, RoundDigits)
            };
        }
        
    }
}