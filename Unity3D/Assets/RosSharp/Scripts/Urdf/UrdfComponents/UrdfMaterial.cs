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
using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEngine;

namespace RosSharp.Urdf
{
    class UrdfMaterial
    {
        private const int RoundDigits = 4;

        public static Dictionary<string, Link.Visual.Material> materials =
            new Dictionary<string, Link.Visual.Material>();

        public static Link.Visual.Material GetMaterialData(Material material)
        {
            if (material == null) return null;

            if (!materials.ContainsKey(material.name))
            {
                if (material.mainTexture != null)
                {
                    Link.Visual.Material.Texture texture = GetTextureData(material.mainTexture);
                    materials[material.name] = new Link.Visual.Material(material.name, null, texture);
                }
                else if (!material.color.Equals(Color.clear))
                {
                    Link.Visual.Material.Color color = new Link.Visual.Material.Color(GetRgba(material));
                    materials[material.name] = new Link.Visual.Material(material.name, color);
                }
                else
                    return null;
            }

            return new Link.Visual.Material(material.name);
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

        private static Link.Visual.Material.Texture GetTextureData(Texture texture)
        {
            string oldTexturePath = AssetDatabase.GetAssetPath(texture);
            string newTexturePath = UrdfExportPathHandler.GetNewResourcePath(Path.GetFileName(oldTexturePath));
            AssetDatabase.CopyAsset(oldTexturePath, newTexturePath);

            string packagePath = UrdfExportPathHandler.GetPackagePathForResource(newTexturePath);
            return new Link.Visual.Material.Texture(packagePath);
        }
    }
}
