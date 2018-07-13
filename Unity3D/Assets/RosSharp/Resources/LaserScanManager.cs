using UnityEngine;
using RosSharp.RosBridgeClient;
using UnityEngine.UI;

public class LaserScanManager : MonoBehaviour
{
    private Renderer _renderer;
    private float _minRange;
    private float _maxRange;
    

    private void Start ()
    {
        _renderer = GetComponent<Renderer>();
        //_renderer.material.shader = Shader.Find("Particles/Additive");
    }
	
    public void SetRanges( float minRange, float maxRange)
    {
        _minRange = minRange;
        _maxRange = maxRange;
    }

 private void Update ()
    {
        UpdateColor(transform.localPosition.magnitude);
	}

    private void UpdateColor(float distance)
    {
        float h_min = (float) 0;
        float h_max = (float) 0.5;

        float h = (float) ( h_min + (distance - _minRange) / (_maxRange - _minRange) * (h_max - h_min));
        float s = (float) 1.0;
        float v = (float) 1.0;

        _renderer.material.color = Color.HSVToRGB(h, s, v);
    }
}
