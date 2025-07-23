using System;
using System.Collections;
using Cinemachine;
using UnityEngine;

public class CarCameraScriptV2 : MonoBehaviour
{

    public float lerpTime = 3.5f;
    public float forwardDistance = 3; private float accelerationEffect;

    public GameObject attachedVehicle; private int locationIndicator = 0; private CarObj controllerRef;

    private Transform target;

    public GameObject focusPoint; public float distance = 2; public Vector2[] cameraPos;
    public bool initializeOffset = false;
    public Vector3 offset;
    public Quaternion targetRotation;
    public Quaternion currentRotation;
    public Vector3 offset1;
    public float rotationDamping;
    public float positionDamping;


    // Start is called before the first frame update
    void Start()
    {

        target = focusPoint.transform;
        controllerRef = attachedVehicle.GetComponent<CarObj>();
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        updateCam();
    }

    private void updateCam()
    {

        if (Input.GetKeyDown(KeyCode.Tab))
        {
            //cycleCam();
        }




        //smoothed g force value
        //accelerationEffect = Mathf. Lerp(accelerationEffect, controllerRef. Gforce * 3.5f, 2 * Time. deltaTime);
        if (!controllerRef.isCarMidAir())

        {

            initializeOffset = false;

                // Calculate target's forward direction
            Vector3 targetForward = target.transform.forward;

            // Calculate target's desired rotation
            targetRotation = Quaternion.LookRotation(targetForward);

            // Smoothly interpolate rotation
            currentRotation = Quaternion.Slerp(currentRotation, targetRotation, rotationDamping * Time.deltaTime);

            // Calculate target position
            Vector3 targetPosition = target.transform.position + offset;

            // Smoothly interpolate position
            transform.position = Vector3.Lerp(transform.position, targetPosition, positionDamping * Time.deltaTime);

            // Apply interpolated rotation and position
            transform.rotation = currentRotation;
            transform.position = target.transform.position + currentRotation * offset;

        }
        else
        {
            if (!initializeOffset)
            {
                offset = new Vector3(attachedVehicle.transform.position.x - transform.position.x, Mathf.Abs(attachedVehicle.transform.position.y - transform.position.y), transform.position.z - attachedVehicle.transform.position.z);
                initializeOffset = true;
            }
            airCam();
        }

    }


    private void airCam()
    {

        // Calculate desired camera position
        Vector3 newPosition = new Vector3(attachedVehicle.transform.position.x + offset.x, attachedVehicle.transform.position.y + offset.y, attachedVehicle.transform.position.z + offset.z);
        transform.position = newPosition;
        Quaternion targetRotation = Quaternion.LookRotation(attachedVehicle.transform.position - transform.position);
        transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, 2*Time.deltaTime);

        //transform.LookAt(attachedVehicle.transform);

        }
}
