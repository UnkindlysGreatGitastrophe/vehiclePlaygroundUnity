using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraScript : MonoBehaviour
{
    float rotationX; float rotationY;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        rotationX += Input.GetAxis("Mouse X");
        rotationY += Input.GetAxis("Mouse Y");

    }

    void FixedUpdate()
    {
        transform.localRotation = Quaternion.Euler(rotationY, rotationX,0);
    }
}
