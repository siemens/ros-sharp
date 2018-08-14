using System;
using UnityEditor;
using UnityEngine;

namespace RosSharp.Urdf
{
    class UrdfMaterial
    {
        private const int RoundDigits = 4;

        public static Link.Visual.Material GetMaterialData(Material material)
        {
            if (material == null) return null;

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
