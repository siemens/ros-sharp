using UnityEngine;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.Services;


public class RosServiceTest : MonoBehaviour
{
    public int a;
    public int b;
    public bool send = false;
    private RosSocket rosSocket;

    void Start()
    {
        rosSocket = gameObject.GetComponent<RosConnector>().RosSocket;
    }

    private void Update()
    {
        if(send)
        {
            CallService();
            send = false;
        }
    }

    public void CallService()
    {
        rosSocket.CallService <AddTwoIntsRequest, AddTwoIntsResponse>
            ("add_two_ints", ServiceCallHandler, new AddTwoIntsRequest(a, b));
    }

    private static void ServiceCallHandler(AddTwoIntsResponse message)
    {
        Debug.Log("Sum: " + message.sum);
    }
}
