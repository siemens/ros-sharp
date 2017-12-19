/*
© Federal Univerity of Minas Gerais (Brazil), 2017
Author: Lucas Coelho Figueiredo (me@lucascoelho.net)

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



using RosSharp.RosBridgeClient;
using System;
using UnityEngine;

namespace RosSharp
{
    public class ImageManager : MonoBehaviour
    {
        private Texture2D imageTexture;
        private bool doUpdate;
        private bool recreateTexture = true;
        private byte[] data;
        private int width;
        private int heigth;
        private TextureFormat encoding;

        private void Start()
        {
            width = 0;
            heigth = 0;
            data = new byte[0];
            recreateTexture = false;
            doUpdate = false;
        }

        private void Update()
        {
            if (recreateTexture)
            {
                imageTexture = new Texture2D(width, heigth, encoding, false);
                imageTexture.LoadRawTextureData(data);
            }
            if (doUpdate)
            {
                imageTexture.Apply();
                GetComponent<Renderer>().material.mainTexture = imageTexture;
                doUpdate = false;
            }
        }

        public void UpdateTexture(SensorImage sensorImage)
        {
            Nullable<TextureFormat> rosEncoding = null;

            rosEncoding = GetEncoding(sensorImage.encoding);

            if(rosEncoding == null) {
                Debug.Log("Encoding " + sensorImage.encoding + " not implemented. Please change to rgb8.");
                return;
            }

            if (sensorImage.width != width || sensorImage.height != heigth || rosEncoding != encoding)
            {
                recreateTexture = true;
                data = new byte[sensorImage.data.Length];
                sensorImage.data.CopyTo(data, 0);
                width = sensorImage.width;
                heigth = sensorImage.height;
                encoding = rosEncoding.Value;
            }
            else
            {
                sensorImage.data.CopyTo(data, 0);
            }
            doUpdate = true;
        }

        private static TextureFormat? GetEncoding(string ros_encoding) 
        {
            if (ros_encoding.Equals("rgb8"))
                return TextureFormat.RGB24;
            else
                return null;
        }
    }
}
