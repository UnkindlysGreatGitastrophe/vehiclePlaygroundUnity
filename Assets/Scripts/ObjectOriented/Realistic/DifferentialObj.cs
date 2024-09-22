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

    public float calculateDifferential(int i)
    {
        float torqueOffset = 0;
        if (difType == DifferntialTypes.LSD)
        {
            car.torqueToWheel = car.Tc * car.gearBox.get_ratio() * differentialFinalGearRatio / poweredWheels;
            torqueOffset = LSDDif(i);
            return torqueOffset;

        }
        if (difType == DifferntialTypes.LOCKED)
        {
            //lockedDif();
            return torqueOffset;

        }
        else
        {
            return torqueOffset;
            // DriveShaftAngularVel += DriveShaftAngularVel + DriveShaftAccel * Time.fixedDeltaTime; // Broken
        }
    }
    

    public float LSDDif(int i)
    {
    // If the difference in road reaction torque (fabs(T_left-T_right)) is bigger than the preload torque, the diff gets unlocked.
    // While the diff is unlocked, you apply the preload torque to counter the wheel velocity difference.
    // Do NOT apply the preload torque back to the engine however! This torque acts directly inside the cage system, and isn't transferred back to the engine.
    // When the sign of the difference in wheel velocities changes, lock the diff. Make sure you understand that sentence; while applying friction, 
    // at some point the difference of the wheel velocities will go from a positive number to a negative number, or vice versa. As soon as this happens, you'll get into the flip-flop state of applying friction. To avoid that, set the diff locked state to true.
         float torqueDiffLR = car.wheels[i].ReactionTorqueToWheel - car.wheels[i+1].ReactionTorqueToWheel; // Torque difference of left - right reaction torque
         float torqueDiffRL = car.wheels[i+1].ReactionTorqueToWheel - car.wheels[i].ReactionTorqueToWheel; // Torque difference of left - right reaction torque
         if (car.wheels[i].ReactionTorqueToWheel / car.wheels[i+1].ReactionTorqueToWheel > torqueBias && torqueDiffLR > preLoadTorque) // Left wheel torque > right wheel torque
             {
                 return -preLoadTorque;
             }
         else if (car.wheels[i+1].ReactionTorqueToWheel / car.wheels[i].ReactionTorqueToWheel > torqueBias && torqueDiffLR > preLoadTorque) // Right wheel torque > Left wheel torque
             {
                 return +preLoadTorque;
             }
         else
             {
                 return 0;
             }
     
    }

    //  public float GetDiffDrag(float torqueFromTransmission, float velocityA, float velocityB, float inertiaA, float inertiaB, float torqueA, float torqueB,float powerAngle, float coastAngle, int clutchPacks, float preload, float delta)
    //     {
    //         float a = torqueB * inertiaA; // Take torque
    //         float b = torqueA * inertiaB;
    //         float c = inertiaA * inertiaB * (velocityA - velocityB) / delta;
      
    //         //torqueFromTransmission it`s torque from the engine * total gear ratio
    //         //Angles must be from 0-90
    //         //Clutch packs from 0-2. 
    //         //Coast must not be larger angle value than Power. It will create unrealistic behaviour
    //         //Ideal power angle is 85
    //         //Ideal coast angle is 45
    //         //Recommended value for clutch packs is 0 or 1.

    //         bool isPowered = Mathf.Sign(torqueFromTransmission) > 0;
    //         float currentDiffRatio = Mathf.Max((isPowered ? Mathf.Cos(powerAngle * Mathf.Deg2Rad) : Mathf.Cos(coastAngle * Mathf.Deg2Rad));

    //         float maxTorqueTransfer = Mathf.Max(preload, currentDiffRatio * (1+2*clutchPacks) * Mathf.Abs(torqueFromTransmission));

    //         float lockTorque = (a - b + c) / (inertiaA + inertiaB);
    //         //Btw, In lock torque u can put locked differential calculation too
    //         return Mathf.Clamp(lockTorque , -maxTorqueTransfer , maxTorqueTransfer);
    //     } 




}
