/*
© Siemens AG, 2024
Author: hoyadong1 (github.com/hoyadong1)

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
using UnityEngine.UI;

public class MapVisualizer : MonoBehaviour
{
    public GameObject rosBridge;
    private RosSharp.RosBridgeClient.MapSubscriber mapSubscriber;
    public RawImage rawImage;
    private Texture2D mapTexture;

    void Start()
    {
        if (rosBridge != null)
        {
            mapSubscriber = rosBridge.GetComponentInChildren<RosSharp.RosBridgeClient.MapSubscriber>();
        }

        if (mapSubscriber == null)
        {
            Debug.LogError("Could not find MapSubscriber. Check RosBridge object.");
        }
    }

    void Update()
    {
        if (mapSubscriber != null && mapSubscriber.isMapReceived)
        {
            GenerateMapTexture();
            mapSubscriber.isMapReceived = false;
        }
    }

    private void GenerateMapTexture()
    {
        int width = mapSubscriber.width;
        int height = mapSubscriber.height;
        sbyte[] mapData = mapSubscriber.mapData;

        if (mapTexture == null || mapTexture.width != width || mapTexture.height != height)
        {
            mapTexture = new Texture2D(width, height, TextureFormat.RGB24, false);
        }

        Color[] pixels = new Color[width * height];
        for (int i = 0; i < mapData.Length; i++)
        {
            float value = mapData[i] == -1 ? 0.5f : mapData[i] / 100f;
            pixels[i] = mapData[i] == -1
                ? Color.gray
                : Color.Lerp(Color.white, Color.black, value);
        }

        mapTexture.SetPixels(pixels);
        mapTexture.Apply();

        rawImage.texture = mapTexture;
    }
}
