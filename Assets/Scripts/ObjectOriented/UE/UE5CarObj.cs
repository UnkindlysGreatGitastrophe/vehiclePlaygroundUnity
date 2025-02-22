using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UE5CarObj : MonoBehaviour
{
    [Header("References")]
    public UE5GearboxObj UEGearbox;
    public UE5ClutchObj UEClutch;
    public UE5DifferentialObj[] UEDifferential;
    public UE5EngineObj UEEngine;
    public enum DriveType {FWD, RWD, AWD};
    [Header("DriveTrain Parameters")]
    [Tooltip("Determines Drivetrain type (RWD, AWD, FWD)")]
    public DriveType driveType = DriveType.RWD; // Determines Drivetrain type (RWD, AWD, FWD?)
    [Tooltip("Distributes torque to the front and rear wheel axles, approach 1 for a rear wheel bias, approach 0 for a front wheel bias (NOTE: this variable is only relevant when the driveType is AWD)")]
    public float torqueDistribution = 0.6f; // Option Applicable when AWD is selected, otherwise the power is distributed only to rear or front wheels.
    [Tooltip("Ratio of torque distribution, first element is for the rear wheels, 2nd element is for the front wheels")]
    public float[] torqueRatio = new float[2]; // Ratio of torque distribution
    public float BrakeDistribution = 0.6f; // Option Applicable when AWD is selected, otherwise the power is distributed only to rear or front wheels.



    [Header("Input")]
    public float throttle = 0;
    public float brake = 0;
    public float brakeTorque = 0f;
    // Start is called before the first frame update
    void Start()
    {
        if (driveType == DriveType.AWD)
        {
            torqueDistribution = Mathf.Clamp(torqueDistribution, 0, 1); // Make sure it is within the correct boundaries
            torqueRatio[0] = torqueDistribution; // Distribute power to rear wheels
            torqueRatio[1] = 1-torqueDistribution; // Then the remainder to the front wheels
        }
        else if (driveType == DriveType.RWD)
        {
            torqueRatio[0] = 1; // Distribute power to rear wheels
            torqueRatio[1] = 0; // Then the remainder to the front wheels
        }
        else if (driveType == DriveType.FWD)
        {
            torqueRatio[0] = 0; // Distribute power to rear wheels
            torqueRatio[1] = 1; // Then the remainder to the front wheels
        }
        BrakeDistribution = Mathf.Clamp(BrakeDistribution, 0, 1); // Make sure it is within the correct boundaries
        UEDifferential[0].brakeRatio = BrakeDistribution;
        UEDifferential[1].brakeRatio = 1-BrakeDistribution;

    }

    // Update is called once per frame
    void FixedUpdate()
    {
        float outputTorque = UEGearbox.GetOuputTorque(UEClutch.clutchTorque);
        for (int i = 0; i < UEDifferential.Length; i++) // Rear Diff goes first
        {
            UEDifferential[0].GetOutputTorque(outputTorque * torqueRatio[i]);
        }
            UEEngine.UpdatePhysic();
        for (int i = 0; i < UEDifferential.Length; i++)
        {
            for (int j = 0; i < UEDifferential[i].UEconnectedWheels.Length; i++)
            UEDifferential[i].UEconnectedWheels[j].UpdatePhysic();
        }

        
        /*
        Differential.GetInputShaftVelocity();
        Gearbox.GetInputShaftVelocity();
        Clutch.UpdatePhysic();
        Engine.UpdatePhysic();
        Clutch.UpdatePhysic();
        */
    }
}
