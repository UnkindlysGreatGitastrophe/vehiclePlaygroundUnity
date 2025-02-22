using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UE5DifferentialObj : MonoBehaviour
{   
    public UE5WheelObj[] UEconnectedWheels;
    public UE5GearboxObj UE5Gearbox;

    public float torqueToAxle;
    internal float brakeRatio;

    internal void GetOutputTorque(float inputTorque)
    {
        torqueToAxle = inputTorque * UE5Gearbox.finalDriveGearRatio;
        
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
