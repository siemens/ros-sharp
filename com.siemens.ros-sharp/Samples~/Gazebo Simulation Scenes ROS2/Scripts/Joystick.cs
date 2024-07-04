using UnityEngine;
using UnityEngine.EventSystems;

namespace RosSharp.RosBridgeClient
{
    public class Joystick : MonoBehaviour, IDragHandler, IPointerUpHandler, IPointerDownHandler
    {
        private RectTransform background;
        private RectTransform handle;
        private Vector2 inputVector;

        [SerializeField]
        private string horizontalAxisName = "Horizontal";

        [SerializeField]
        private string verticalAxisName = "Vertical";

        public string HorizontalAxisName
        {
            get { return horizontalAxisName; }
            private set { horizontalAxisName = value; }
        }

        public string VerticalAxisName
        {
            get { return verticalAxisName; }
            private set { verticalAxisName = value; }
        }

        private void Start()
        {
            background = GetComponent<RectTransform>();
            handle = transform.GetChild(0).GetComponent<RectTransform>();
        }

        public void OnDrag(PointerEventData eventData)
        {
            Vector2 pos;
            if (RectTransformUtility.ScreenPointToLocalPointInRectangle(background, eventData.position, eventData.pressEventCamera, out pos))
            {
                pos.x = pos.x / background.sizeDelta.x;
                pos.y = pos.y / background.sizeDelta.y;

                inputVector = new Vector2(pos.x * 2, pos.y * 2);
                inputVector = (inputVector.magnitude > 1.0f) ? inputVector.normalized : inputVector;

                handle.anchoredPosition = new Vector2(inputVector.x * (background.sizeDelta.x / 2), inputVector.y * (background.sizeDelta.y / 2));
            }
        }

        public void OnPointerDown(PointerEventData eventData)
        {
            OnDrag(eventData);
        }

        public void OnPointerUp(PointerEventData eventData)
        {
            inputVector = Vector2.zero;
            handle.anchoredPosition = Vector2.zero;
        }

        public float Horizontal()
        {
            return inputVector.x;
        }

        public float Vertical()
        {
            return inputVector.y;
        }
    }
}
