using System;
using UnityEngine.Assertions; 
using Baracuda.Monitoring;
using UnityEngine;

public class DifferentialObj : MonoBehaviour
{
    public enum DifferntialTypes {OPEN, LOCKED, LSD};

    [Header("References")]
    public CarObj car;
    public WheelObj[] connectedWheels;
    [Header("DriveShaft Inputs")]
    public float DriveShaftInertia = 5;

    [Header("DriveShaft Outputs")]
    public float DriveShaftTorque = 0;
    public float DriveShaftAccel = 0;
    public float DriveShaftAngularVel = 0;

    [Header("Differential Variables")]
    public DifferntialTypes difType = DifferntialTypes.LSD;
    public float preLoadTorque = 50f;
    public float powerAngle;
    public float coastAngle;
    public int clutchPacks;

    [Header("Differential Output")]

    public bool isPowered;
    public float currentDiffRatio;
    public float maxTorqueTransfer;
    public float lockTorque;

    [Monitor] public float torqueToWheel;






    // Start is called before the first frame update
    void Start()
    {
        this.StartMonitoring();
        
        connectedWheels = new WheelObj[transform.GetChild(0).childCount];
        Assert.AreEqual(2, connectedWheels.Length);

        for (int i = 0; i < connectedWheels.Length; i++)
        {
            connectedWheels[i] = transform.GetChild(0).GetChild(i).GetComponent<WheelObj>();
        }

    }

    public void calculateDifferential() // Each powered axle has a differential, we calculate how we spread the torque in accordance to the differential type here. i is the index of the differential
    {
        if (difType == DifferntialTypes.LSD) // If the dif type is a limited slip differential, then we use a salisbury LSD approach to distributing torque correctly
        {
            GetDiffDrag(difType);
        }
        else if (difType == DifferntialTypes.LOCKED) // If Dif type is locked, we limit the top speed differences as much as possible.
        {
            GetDiffDrag(difType);
        }
        else
        {
            torqueToWheel = car.torqueToAxle/2;
            connectedWheels[0].applyTorqueToWheels(torqueToWheel); // Apply torque "evenly", the consequence is that one wheel spins fastest, due to having the least friction compared to every other car.
            connectedWheels[1].applyTorqueToWheels(torqueToWheel);
            // DriveShaftAngularVel += DriveShaftAngularVel + DriveShaftAccel * Time.fixedDeltaTime; // Broken
        }
    }
    


     public void GetDiffDrag( DifferntialTypes differentialType)
        {

      
            //torqueFromTransmission it`s torque from the engine * total gear ratio
            //Angles must be from 0-90
            //Clutch packs from 0-2. 
            //Coast must not be larger angle value than Power. It will create unrealistic behaviour
            //Ideal power angle is 85
            //Ideal coast angle is 45
            //Recommended value for clutch packs is 0 or 1.

            if (differentialType == DifferntialTypes.LSD)
            {
                //The following 3 variables are based off the clutch derivation formula from eariler
                //float a = torqueB * inertiaA; // Take torque
                //float b = torqueA * inertiaB;
                //float c = inertiaA * inertiaB * (velocityA - velocityB) / delta;
                float a = connectedWheels[1].ReactionTorqueToWheel * connectedWheels[0].wheelInertia;
                float b = connectedWheels[0].ReactionTorqueToWheel * connectedWheels[1].wheelInertia;
                float c = connectedWheels[0].wheelInertia * connectedWheels[1].wheelInertia * (connectedWheels[0].wheelAngularVelocity - connectedWheels[1].wheelAngularVelocity) / Time.fixedDeltaTime;
                isPowered = Math.Sign(car.torqueToAxle) > 0;
                currentDiffRatio = Mathf.Max(isPowered ? Mathf.Cos(powerAngle * Mathf.Deg2Rad) : Mathf.Cos(coastAngle * Mathf.Deg2Rad)); // This is where the salisbury comes in, due to the design, we are using cosine angles to get the diff ratio
                maxTorqueTransfer = Mathf.Max(preLoadTorque, currentDiffRatio * (1+2*clutchPacks) * Mathf.Abs(car.torqueToAxle)); // This is what we use to clamp the offset torque to add in to the clutch toque

                lockTorque = (a - b + c) / (connectedWheels[0].wheelInertia + connectedWheels[1].wheelInertia); // The algebraic equation for solving the locktorque required to even out the wheels.
                lockTorque = Mathf.Clamp(lockTorque , -maxTorqueTransfer , maxTorqueTransfer);
                connectedWheels[0].applyTorqueToWheels(car.torqueToAxle-lockTorque);
                connectedWheels[1].applyTorqueToWheels(car.torqueToAxle+lockTorque);
            }
            else
            {
                // This simply tries to keep the wheel differences as tight as possible, needs a low substep to work right
                float halfAngularVel = (connectedWheels[0].wheelAngularVelocity - connectedWheels[0+1].wheelAngularVelocity) * 0.5f / Time.fixedDeltaTime;
                float lockedTorque = halfAngularVel * connectedWheels[0].wheelInertia; 
                connectedWheels[0].applyTorqueToWheels(car.torqueToAxle * 0.5f - lockedTorque);
                connectedWheels[1].applyTorqueToWheels(car.torqueToAxle * 0.5f + lockedTorque);

            }
            
        } 




}
