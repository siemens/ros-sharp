using System;
using WebSocketSharp;

namespace RosSharp.RosBridgeClient.Protocols
{
    public class WebsocketProtocol: IProtocol
    {
        public event EventHandler OnMessage;

        private WebSocket WebSocket;

        public WebsocketProtocol(string url)
        {
            WebSocket = new WebSocket(url);
            WebSocket.OnMessage += OnWebsocketMessage;                
        }
                
        public void Connect()
        {
            WebSocket.Connect();            
        }

        public void Close()
        {
            WebSocket.Close();
        }

        public bool IsAlive()
        {
            return WebSocket.IsAlive;
        }

        public void SendAsync(byte[] data, Action<bool> completed)
        {
            WebSocket.SendAsync(data, completed);
        }
        
        private void OnWebsocketMessage(object sender, WebSocketSharp.MessageEventArgs e)
        {            
            OnMessage.Invoke(sender, new MessageEventArgs(e.RawData));
        }
    }
}
