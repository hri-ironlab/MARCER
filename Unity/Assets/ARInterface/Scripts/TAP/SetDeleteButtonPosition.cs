using UnityEngine;
using TMPro;


public class SetDeleteButtonPosition : MonoBehaviour
{
    [SerializeField] private TextMeshPro _text;
    [SerializeField] GameObject _button;
    [SerializeField] private float _offset = .032f; 

    private void Start()
    {
        
    }
    private void Update()
    {
        if (_text.bounds.extents.x > 0.0)
        {
            UpdatePosition();
            enabled = false; 
        }
    }

    public void UpdatePosition()
    {
        Bounds textBounds = _text.bounds;
        _button.transform.localPosition = new Vector3(textBounds.center.x + textBounds.extents.x + _offset, 0.0f, 0.0f);
    }
}
