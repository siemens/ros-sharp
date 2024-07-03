/*
© Siemens AG, 2024
Author: Mehmet Emre Cakal (emre.cakal@siemens.com / m.emrecakal@gmail.com)

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

namespace RosSharp.RosBridgeClient
{
    [RequireComponent(typeof(RosConnector))]
    public class RawImageSubscriber : UnitySubscriber<MessageTypes.Sensor.Image>
    {
        public MeshRenderer meshRenderer;

        private Texture2D texture2D;
        private byte[] imageData;
        private bool isMessageReceived;
        private int width = 1;
        private int height = 1;
        private int messageCount = 0;
        private const int checkThreshold = 10;

        protected override void Start()
        {
            base.Start();
            texture2D = new Texture2D(width, height, TextureFormat.RGB24, false);
            meshRenderer.material = new Material(Shader.Find("Standard"));
        }

        private void Update()
        {
            if (isMessageReceived)
                ProcessMessage();
        }

        protected override void ReceiveMessage(MessageTypes.Sensor.Image image)
        {
            if (messageCount <= checkThreshold)
            {
                width = (int)image.width;
                height = (int)image.height;
            }

            imageData = image.data;
            isMessageReceived = true;
        }

        private void ProcessMessage()
        {
            if (messageCount < checkThreshold)
            {

                if (texture2D.width != width || texture2D.height != height)
                {
                    texture2D.Reinitialize(width, height);
                    Debug.Log("Texture size reinitialized to " + width + "x" + height);
                }

                messageCount++;
            }

            // Load raw image data into the texture
            texture2D.LoadRawTextureData(imageData);
            texture2D.Apply();

            // Set the texture to the MeshRenderer
            meshRenderer.material.SetTexture("_MainTex", texture2D);
            isMessageReceived = false;
        }
    }
}
