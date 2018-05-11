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

using System;

namespace RosSharp.RosBridgeClient
{
    public class PointCloud
    {
        public Point[] Points;

        public PointCloud(SensorPointCloud2 sensorPointCloud2)
        {
            int I = sensorPointCloud2.data.Length / sensorPointCloud2.point_step;
            Points = new Point[I];
            byte[] byteSlice = new byte[sensorPointCloud2.point_step];
            for (int i = 0; i < I; i++)
            {
                Array.Copy(sensorPointCloud2.data, i * sensorPointCloud2.point_step, byteSlice, 0, sensorPointCloud2.point_step);
                Points[i] = new Point(byteSlice);
            }
        }


        public PointCloud(SensorImage depthImage, SensorImage rgbImage, float focal)
        {

            int width = depthImage.width;
            int height = depthImage.height;
            float invFocal = 1.0f / focal;

            Points = new Point[width * height];

            for (int v = 0; v < height; v++)
            {
                for (int u = 0; u < width; u++)
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
                        Points[u + v * width].rgb = new int[] { 0, 0, 0 };// rgbImage[u,v];
                    }
                }
            }
        }
    }
    public class Point
        {
            public float x;
            public float y;
            public float z;
            public int[] rgb;

            public Point(byte[] bytes)
            {
                byte[] slice = new byte[4];
                Array.Copy(bytes, 0, slice, 0, 4);
                x = getValue(slice);
                Array.Copy(bytes, 4, slice, 0, 4);
                y = getValue(slice);
                Array.Copy(bytes, 8, slice, 0, 4);
                z = getValue(slice);
                Array.Copy(bytes, 16, slice, 0, 4);
                rgb = getRGB(slice);
            }

            public override string ToString()
            {
                return "xyz=(" + x.ToString() + ", " + y.ToString() + ", " + z.ToString() + ")"
                    + "  rgb=(" + rgb[0].ToString() + ", " + rgb[1].ToString() + ", " + rgb[2].ToString() + ")";
            }
            private static float getValue(byte[] bytes)
            {
                if (!BitConverter.IsLittleEndian)
                    Array.Reverse(bytes);

                float result = BitConverter.ToSingle(bytes, 0);
                return result;
            }
            private static int[] getRGB(byte[] bytes)
            {
                int[] rgb = new int[3];
                rgb[0] = Convert.ToInt16(bytes[0]);
                rgb[1] = Convert.ToInt16(bytes[1]);
                rgb[2] = Convert.ToInt16(bytes[2]);
                return rgb;
            }
        }

    }
