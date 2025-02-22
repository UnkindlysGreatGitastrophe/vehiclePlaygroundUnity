using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UE5GearboxObj : MonoBehaviour
{
    private float ratio;
    public float finalDriveGearRatio;

    internal float GetOuputTorque(float inputTorque)
    {
        return inputTorque * ratio;
    }


    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
