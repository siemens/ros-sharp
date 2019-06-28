/*
© Siemens AG, 2017-2019
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

using System;
using RosSharp.RosBridgeClient.Messages.Sensor;

namespace RosSharp.RosBridgeClient
{
    public class PointCloud
    {
        public RgbPoint3[] Points;

        public PointCloud(PointCloud2 pointCloud2)
        {
            long I = pointCloud2.data.Length / pointCloud2.point_step;
            Points = new RgbPoint3[I];
            byte[] byteSlice = new byte[pointCloud2.point_step];
            for (long i = 0; i < I; i++)
            {
                Array.Copy(pointCloud2.data, i * pointCloud2.point_step, byteSlice, 0, pointCloud2.point_step);
                Points[i] = new RgbPoint3(byteSlice, pointCloud2.fields);
            }
        }

        public PointCloud(Image depthImage, Image rgbImage, float focal)
        {

            uint width = depthImage.width;
            uint height = depthImage.height;
            float invFocal = 1.0f / focal;

            Points = new RgbPoint3[width * height];

            for (uint v = 0; v < height; v++)
            {
                for (uint u = 0; u < width; u++)
                {
                    float depth = 0;// depthImage[u, v];
                    if (depth == 0)
                    {
                        Points[u + v * width].x = float.NaN;
                        Points[u + v * width].y = float.NaN;
                        Points[u + v * width].z = float.NaN;
                        Points[u + v * width].rgb = new int[] { 0, 0, 0 };
                    }
                    else
                    {
                        Points[u + v * width].z = depth * invFocal;
                        Points[u + v * width].x = u * depth * invFocal;
                        Points[u + v * width].y = v * depth * invFocal;
                        Points[u + v * width].rgb = new int[] { 0, 0, 0 };// rgbImage[u, v];
                    }
                }
            }
        }
    }
    public class RgbPoint3
    {
        public float x;
        public float y;
        public float z;
        public int[] rgb;

        public RgbPoint3(byte[] bytes, PointField[] fields)
        {
            foreach (var field in fields)
            {
                byte[] slice = new byte[field.count * 4];
                Array.Copy(bytes, field.offset, slice, 0, field.count * 4);

                switch (field.name)
                {
                    case "x":
                        x = GetValue(slice);
                        break;
                    case "y":
                        y = GetValue(slice);
                        break;
                    case "z":
                        z = GetValue(slice);
                        break;
                    case "rgb":
                        rgb = GetRGB(slice);
                        break;
                }
            }
        }

        public override string ToString()
        {
            return "xyz=(" + x.ToString() + ", " + y.ToString() + ", " + z.ToString() + ")"
                + "  rgb=(" + rgb[0].ToString() + ", " + rgb[1].ToString() + ", " + rgb[2].ToString() + ")";
        }
        private static float GetValue(byte[] bytes)
        {
            if (!BitConverter.IsLittleEndian)
                Array.Reverse(bytes);

            float result = BitConverter.ToSingle(bytes, 0);
            return result;
        }
        private static int[] GetRGB(byte[] bytes)
        {
            int[] rgb = new int[3];
            rgb[0] = Convert.ToInt16(bytes[0]);
            rgb[1] = Convert.ToInt16(bytes[1]);
            rgb[2] = Convert.ToInt16(bytes[2]);
            return rgb;
        }
    }
}
