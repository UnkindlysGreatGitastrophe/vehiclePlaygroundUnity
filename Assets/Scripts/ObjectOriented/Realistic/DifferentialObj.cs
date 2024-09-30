using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class DifferentialObj : MonoBehaviour
{
    public enum DifferntialTypes {OPEN, LOCKED, LSD};
    public enum DriveType {FWD, RWD, AWD};

    [Header("References")]
    public CarObj car;
    [Header("DriveShaft Inputs")]
    public float DriveShaftInertia = 5;
    public float differentialFinalGearRatio = 5;


    [Header("DriveShaft Outputs")]
    public float DriveShaftTorque = 0;
    public float DriveShaftAccel = 0;
    public float DriveShaftAngularVel = 0;

    [Header("Differential Variables")]
    public DifferntialTypes difType = DifferntialTypes.LSD;
    public DriveType driveType = DriveType.RWD;
    public float preLoadTorque = 50f;
    public float torqueBias = 1.5f; // 150:100

    public int poweredWheels;
    public float powerAngle;
    public float coastAngle;
    public int clutchPacks;






    // Start is called before the first frame update
    void Start()
    {
        if (driveType == DriveType.AWD)
        {
            poweredWheels = car.wheels.Length;
        }
        else
        {
            poweredWheels = 2;
        }
    }

    public void calculateDifferential(int i)
    {
        if (difType == DifferntialTypes.LSD)
        {
            car.torqueToWheel = car.Tc * car.gearBox.get_ratio() * differentialFinalGearRatio / poweredWheels;
            GetDiffDrag(i,difType);
        }
        if (difType == DifferntialTypes.LOCKED)
        {
            GetDiffDrag(i,difType);
        }
        else
        {
            car.torqueToWheel = car.Tc * car.gearBox.get_ratio() * differentialFinalGearRatio / poweredWheels;
            car.poweredWheels[i].applyTorqueToWheels(car.torqueToWheel);
            car.poweredWheels[i+1].applyTorqueToWheels(car.torqueToWheel);
            // DriveShaftAngularVel += DriveShaftAngularVel + DriveShaftAccel * Time.fixedDeltaTime; // Broken
        }
    }
    


     public void GetDiffDrag(int i, DifferntialTypes differentialType)
        {
            float a = car.poweredWheels[i+1].ReactionTorqueToWheel * car.poweredWheels[i].wheelInertia;
            float b = car.poweredWheels[i].ReactionTorqueToWheel * car.poweredWheels[i+1].wheelInertia;
            float c = car.poweredWheels[i].wheelInertia * car.poweredWheels[i+1].wheelInertia * (car.poweredWheels[i].wheelAngularVelocity - car.poweredWheels[i+1].wheelAngularVelocity) / Time.fixedDeltaTime;

         //The following 3 variables are based off the clutch derivation formula from eariler
            //float a = torqueB * inertiaA; // Take torque
            //float b = torqueA * inertiaB;
            //float c = inertiaA * inertiaB * (velocityA - velocityB) / delta;
      
            //torqueFromTransmission it`s torque from the engine * total gear ratio
            //Angles must be from 0-90
            //Clutch packs from 0-2. 
            //Coast must not be larger angle value than Power. It will create unrealistic behaviour
            //Ideal power angle is 85
            //Ideal coast angle is 45
            //Recommended value for clutch packs is 0 or 1.

            if (differentialType == DifferntialTypes.LSD)
            {
                bool isPowered = Mathf.Sign(car.torqueToWheel) > 0;
                float currentDiffRatio = Mathf.Max(isPowered ? Mathf.Cos(powerAngle * Mathf.Deg2Rad) : Mathf.Cos(coastAngle * Mathf.Deg2Rad));

                float maxTorqueTransfer = Mathf.Max(preLoadTorque, currentDiffRatio * (1+2*clutchPacks) * Mathf.Abs(car.torqueToWheel));

                float lockTorque = (a - b + c) / (car.poweredWheels[i].wheelInertia + car.poweredWheels[i+1].wheelInertia);
                //Btw, In lock torque u can put locked differential calculation too
                lockTorque = Mathf.Clamp(lockTorque , -maxTorqueTransfer , maxTorqueTransfer);
                car.poweredWheels[i].applyTorqueToWheels(car.torqueToWheel-lockTorque);
                car.poweredWheels[i+1].applyTorqueToWheels(car.torqueToWheel+lockTorque);
            }
            else
            {
                float halfAngularVel = (car.poweredWheels[i].wheelAngularVelocity - car.poweredWheels[i+1].wheelAngularVelocity) * 0.5f / Time.fixedDeltaTime;
                float lockedTorque = halfAngularVel * car.poweredWheels[i].wheelInertia;
                car.poweredWheels[i].applyTorqueToWheels(car.torqueToWheel * 0.5f - lockedTorque);
                car.poweredWheels[i+1].applyTorqueToWheels(car.torqueToWheel * 0.5f + lockedTorque);

            }
            
        } 




}
