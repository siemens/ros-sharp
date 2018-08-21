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
                    Link.Visual.Material.Texture texture = GetTexture(material.mainTexture);
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

        private static Link.Visual.Material.Texture GetTexture(Texture texture)
        {
            string oldTexturePath = AssetDatabase.GetAssetPath(texture);
            string newTexturePath = UrdfAssetPathHandler.GetNewResourcePath(Path.GetFileName(oldTexturePath));
            Directory.CreateDirectory(Path.GetDirectoryName(newTexturePath));
            AssetDatabase.CopyAsset(oldTexturePath, UrdfAssetPathHandler.GetRelativeAssetPath(newTexturePath));

            string packagePath = UrdfAssetPathHandler.GetPackagePathForResource(AssetDatabase.GetAssetPath(texture));
            return new Link.Visual.Material.Texture(packagePath);
        }
    }
}
