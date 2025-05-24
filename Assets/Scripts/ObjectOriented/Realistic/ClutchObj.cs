using System;
using System.Collections;
using System.Collections.Generic;
using Baracuda.Monitoring;
using UnityEngine;

public class ClutchObj : MonoBehaviour
{
    // Start is called before the first frame update
    [Header("References")]
    public CarObj car;

    [Header("Clutch Lock")]
    [Tooltip("Enables manual clutch management")]
    public bool toggleManualClutch = false; // Largely Experimental
    [Tooltip("This represents that amount of connection the engine has with the transmission via the clutch plate. Bounded by 0 <= clutchLock <= 1, where 0 represents no connection and 1 represents full connection")]
    public float clutchLock; // Range is 0 (Disconnected) to 1, fully connected to engine and transmission


    [Header("Clutch B1 (Connected to Engine)")]
    [Tooltip("The Engine's Angular Velocity")]

    [SerializeField] private float w1;
    [Tooltip("The Engine's Output Torque")]
    [SerializeField] private float t1;
    [Tooltip("The Engine's Moment of Inertia")]
    [SerializeField] private float i1;

    [Header("Clutch B2 (Connected to DriveShaft)")]
    [Tooltip("Average Angular Velocity of the powered wheels")]
    [SerializeField] private float w2;
    [Tooltip("The Total Reaction Torque provided by the Powered Wheels of the car")]
    [SerializeField] private float t2;
    [Tooltip("The Moment of Inertia of the Transmission")]
    [SerializeField] private float i2;

    [Header("Clutch V2 parameters")]
    [Tooltip("Torque capacity. It is the amount of torque that can be transmitted by the clutch when it's slipping or fully closed.")]
    [SerializeField] internal float clutchCapacity = 1.3f; // The capacity factor of the clutch, contributes to the max torque a clutch can manage
    [Tooltip("The stiffness of the clutch.")]
    [SerializeField] internal float clutchStiffness = 500f; // The Stiffness of the clutch, also contributes to the max torque.
    [Tooltip("Maximum amount of torque that the clutch can feed into the engine and gearbox, it is equal to: clutch stiffness * clutch capacity")]
    [SerializeField] internal float clutchMaxTorq;


    [Header("Constants")]
    [SerializeField] private float AV_2_RPM = 60 / (2*Mathf.PI);
    [SerializeField] private float RPM_2_AV;

    void Start()
    {
        this.StartMonitoring();
        if (transform.parent.GetComponent<CarObj>() != null)
        {
            car = transform.parent.GetComponent<CarObj>();
        }
        if (transform.parent.parent.GetComponent<CarObj>() != null)
        {
            car = transform.parent.parent.GetComponent<CarObj>();
        }
        clutchMaxTorq = clutchStiffness * clutchCapacity; // What is the very most torque that we can feed into the gearbox?
    }

    public float calculateClutch()
    {
        if (!toggleManualClutch) // If clutch is being operated automatically
        {
            clutchLock = Remap(car.engine.engineAngularVelocity * AV_2_RPM, 1000, 1300, 0,1); // Set the clutch depending on which rpm we are at (1 == Fully connected)
            if (car.gearBox.get_ratio() == 0) // Clutch Lock is automatically disengaged if set to neutral (Note that usually the gears don't touch each other, but the clutch stays on, this is sort of a hacky shortcut)
            {
                clutchLock = 0.0f;
            }
            else // Otherwise update clutch lock based on remap value
            {
                clutchLock = Mathf.Clamp(clutchLock,0.0f, 1.0f);
            }
        }
        
        w1 = car.engine.engineAngularVelocity; // Engine Angular Velocity (RAD/SEC)
        t1 = car.engine.torque_out; // Engine Torque (Nm == kg*m^2/s^2)
        i1 = car.engine.engineMoment; // Engine Flywheel Inertia (kg·m^2)

        t2 = 0; // Wheel Reaction Torque of all POWERED WHEELS!
        
        for (int i = 0; i < car.poweredWheels.Length; i++)
        {
            t2 += car.poweredWheels[i].ReactionTorqueToWheel / (car.gearBox.get_ratio() * car.gearBox.finalDriveGear); // We want to divide by gear ratio and final drive ratio in order to accurately calculate the reaction torque.
            // To be more detailed, since the clutch is placed after the engine and before the gearbox, we cannot have the ratios affecting t2 because the torque coming back from the wheels have lost their torque from passing the diff and gearbox/
        }
        // DriveTrain Inertia
        i2 = (car.poweredWheels[0].wheelInertia*car.poweredWheels.Length)/Mathf.Pow(car.gearBox.get_ratio() * car.gearBox.finalDriveGear,2); // DriveTrain Inertia powered wheels, I2=(wi1+wi2+ ... + wiN)/(gear ratio^2)
        
        float avgSpin = 0;
        for (int i = 0; i < car.poweredWheels.Length; i++) // Take the average wheelangularvelocity of all POWERED wheels
        {
            avgSpin += car.poweredWheels[i].wheelAngularVelocity / car.poweredWheels.Length;
        }

        w2 = avgSpin * (car.gearBox.get_ratio() * car.gearBox.finalDriveGear); // Drivetrain Angular Velocity

			//return (float)Math.Tanh((w1 - w2) * 1.0f) * clutchLock * clutchMaxTorq;
        return Mathf.Clamp( // This is the derivation for finding TC (Torque Clutch)
           -(i1 * t2 - i2 * t1 - i1 * i2 * (w1 - w2) / Time.fixedDeltaTime) / (i1 + i2),
           -clutchMaxTorq * clutchLock,
           clutchMaxTorq * clutchLock);
    }
    

    public static float Remap (float input, float rangeMin, float rangeEnd, float newRangeMin,  float newRangeEnd)
    {
        float t = Mathf.InverseLerp(rangeMin, rangeEnd, input);
        float output = Mathf.Lerp(newRangeMin, newRangeEnd, t);
        return output;
    }

    void Update()
    {
        if (toggleManualClutch)
        {
            if (Input.GetKey(KeyCode.C))
            {
                clutchLock = 0;
            }
            else
            {
                clutchLock = 1;
            }
        }
        
    }
}
