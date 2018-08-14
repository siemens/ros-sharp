/*
© Siemens AG, 2017
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)

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

using UnityEngine;
using System.Collections.Generic;
using System.IO;

namespace RosSharp
{
    public static class StlImporter
    {
        public static Mesh[] Import(string path)
        {
            return IsBinary(path) ? ImportBinary(path) : ImportAscii(path);
        }

        #region Binary

        private static Mesh[] ImportBinary(string path)
        {
            Facet[] facets;

            using (FileStream fileStream = new FileStream(path, FileMode.Open, FileAccess.Read))
            {
                using (BinaryReader binaryReader = new BinaryReader(fileStream, new System.Text.ASCIIEncoding()))
                {
                    binaryReader.ReadBytes(80); // header

                    uint facetCount = binaryReader.ReadUInt32();
                    facets = new Facet[facetCount];

                    for (uint i = 0; i < facetCount; i++)
                        facets[i] = binaryReader.GetFacet();
                }
                return CreateMesh(facets);
            }
        }

        private static Facet GetFacet(this BinaryReader binaryReader)
        {
            Facet facet = new Facet();
            facet.normal = binaryReader.GetVector3();

            facet.vertices = new Vector3[3];
            for (int i = 0; i < 3; i++)
                facet.vertices[i] = binaryReader.GetVector3();

            binaryReader.ReadUInt16(); // padding

            return facet;
        }
        private static Vector3 GetVector3(this BinaryReader binaryReader)
        {
            Vector3 vector3 = new Vector3();
            for (int i = 0; i < 3; i++)
                vector3[i] = binaryReader.ReadSingle();

            return vector3.UnityCoordTrafo();
        }

        #endregion

        #region Ascii

        private static Mesh[] ImportAscii(string path)
        {
            List<Facet> facets = new List<Facet>();
            Facet facet = new Facet();
            using (StreamReader streamReader = new StreamReader(path))
                while ((facet = GetFacet(streamReader)) != null)
                {
                    facets.Add(facet);
                }
            return CreateMesh(facets);
        }

        private static Facet GetFacet(this StreamReader streamReader)
        {
            string line;
            Facet facet = new Facet();
            while ((line = streamReader.ReadLine()) != null)
            {
                line = line.Trim();
                if (line.StartsWith("facet"))
                {
                    facet.normal = GetVector3(line.Substring(13));
                    facet.vertices = streamReader.GetVertices();
                    return facet;
                }
            }
            return null;
        }

        private static Vector3[] GetVertices(this StreamReader streamReader)
        {
            Vector3[] vertices = new Vector3[3];
            int i = 0;
            string line;
            while ((line = streamReader.ReadLine()) != null)
            {
                line = line.Trim();
                if (line.StartsWith("vertex"))
                    vertices[i++] = GetVector3(line.Replace("vertex ", ""));
                if (i == 3)
                    return vertices;
            }
            return null;
        }
        private static Vector3 GetVector3(string _string)
        {
            string[] strings = _string.Trim().Split();

            Vector3 vector3 = new Vector3();

            float.TryParse(strings[0], out vector3.x);
            float.TryParse(strings[1], out vector3.y);
            float.TryParse(strings[2], out vector3.z);

            return vector3.UnityCoordTrafo();
        }

        #endregion

        #region Common

        private class Facet
        {
            public Vector3 normal;
            public Vector3[] vertices;
        }

        private static Vector3 UnityCoordTrafo(this Vector3 vector3)
        {
            return new Vector3(-vector3.y, vector3.z, vector3.x);
        }

        private static bool IsBinary(string path)
        {
            int maxCharsToCheck = 100;

            using (StreamReader reader = new StreamReader(path))
                for (int i = 0; i < maxCharsToCheck; i++)
                    if (reader.Read() == '\0')
                        return true;

            return false;
        }

        private static Mesh[] CreateMesh(IList<Facet> facets)
        {
            int maxVerticesPerMesh = 65535;
            int totalNumberOfFacets = facets.Count;
            int totalFacetIndex = 0;
            int[] order = new int[] { 0, 2, 1 };

            Mesh[] meshes = new Mesh[totalNumberOfFacets / (maxVerticesPerMesh / 3) + 1];
            Vector3[] vertices;
            Vector3[] normals;
            int[] triangles;

            for (int meshIndex = 0; meshIndex < meshes.Length; meshIndex++)
            {
                int meshSize = Mathf.Min(maxVerticesPerMesh, (totalNumberOfFacets - totalFacetIndex) * 3);

                vertices = new Vector3[meshSize];
                normals = new Vector3[meshSize];
                triangles = new int[meshSize];
                for (int facetIndex = 0; facetIndex < meshSize; facetIndex += 3)
                {
                    for (int vertexIndex = 0; vertexIndex < 3; vertexIndex++)
                    {
                        vertices[facetIndex + vertexIndex] = facets[totalFacetIndex].vertices[order[vertexIndex]];
                        normals[facetIndex + vertexIndex] = facets[totalFacetIndex].normal;
                        triangles[facetIndex + vertexIndex] = facetIndex + vertexIndex;
                    }
                    totalFacetIndex++;
                }

                meshes[meshIndex] = new Mesh();
                meshes[meshIndex].vertices = vertices;
                meshes[meshIndex].normals = normals;
                meshes[meshIndex].triangles = triangles;
            }
            return meshes;
        }
        #endregion
    }
}