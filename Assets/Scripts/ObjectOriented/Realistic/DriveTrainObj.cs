using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DriveTrainObj : MonoBehaviour
{
    [Header("References")]
    public CarObj car;
    [Header("DriveShaft Inputs")]
    public float DriveShaftInertia = 5;
    public float differentialFinalGearRatio = 5;


    [Header("DriveShaft Outputs")]
    public float DriveShaftTorque = 0;
    public float DriveShaftAccel = 0;
    public float DriveShaftAngularVel = 0;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    public void calculateDriveShaft()
    {
        DriveShaftTorque = car.Tc * car.gearBox.get_ratio();
        DriveShaftAccel = DriveShaftTorque / DriveShaftInertia;
        // DriveShaftAngularVel += DriveShaftAngularVel + DriveShaftAccel * Time.fixedDeltaTime; // Broken
    }


}
