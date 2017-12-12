using RosSharp.RosBridgeClient;
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
            TextureFormat rosEncoding;
            try
            {
                rosEncoding = GetEncoding(sensorImage.encoding);
            }
            catch (System.NotImplementedException)
            {
                Debug.Log("Image encoding " + sensorImage.encoding + " not supported. Please change it to rgb8 if possible.");
                return;
            }
            if (sensorImage.width != width || sensorImage.height != heigth || rosEncoding != encoding)
            {
                recreateTexture = true;
                data = new byte[sensorImage.data.Length];
                sensorImage.data.CopyTo(data, 0);
                width = sensorImage.width;
                heigth = sensorImage.height;
                encoding = rosEncoding;
            }
            else
            {
                sensorImage.data.CopyTo(data, 0);
            }
            doUpdate = true;
        }

        public static TextureFormat GetEncoding(string ros_encoding) 
        {
            if (ros_encoding.Equals("rgb8"))
            {
                return TextureFormat.RGB24;
            }
            else
            {
                throw new System.NotImplementedException();
            }
            /** TODO: Add more encoding types, I added just the one I needed, which is the most common encoding type 
             *        When adding more types, we might also need to convert them to a format which Unity can understand,
             *        so it will be necessary to pass the sensorImage.data too for conversion.
             *        Unity does not read BGR24 format by default. **/
        }
    }
}
