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
    public bool toggleManualClutch = false;
    [Monitor] public float clutchLock;


    [Header("Clutch B1 (Connected to Engine)")]
    [SerializeField] private float w1;
    [SerializeField] private float t1;
    [SerializeField] private float i1;

    [Header("Clutch B2 (Connected to DriveShaft)")]
    [SerializeField] private float w2;
    [SerializeField] private float t2;
    [SerializeField] private float i2;

    [Header("Clutch V2 parameters")]

    [SerializeField] private float clutchTorq;
    [SerializeField] private float clutchDampening = 0.7f;
    [SerializeField] private float clutchCapacity = 1.3f;
    [SerializeField] private float clutchStiffness = 500f;
    [SerializeField] private float clutchMaxTorq;


    [Header("Constants")]
    [SerializeField] private float AV_2_RPM = 60 / (2*Mathf.PI);
    [SerializeField] private float RPM_2_AV;

    void Start()
    {
        this.StartMonitoring();
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
            t2 += car.poweredWheels[i].ReactionTorqueToWheel / (car.gearBox.get_ratio() * car.differential[0].differentialFinalGearRatio); // We want to divide by gear ratio and final drive ratio in order to accurately calculate the reaction torque.
            // To be more detailed, since the clutch is placed after the engine and before the gearbox, we cannot have the ratios affecting t2 because the torque coming back from the wheels have lost their torque from passing the diff and gearbox/
        }
        // DriveTrain Inertia
        i2 = (car.poweredWheels[0].wheelInertia*car.poweredWheels.Length)/Mathf.Pow(car.gearBox.get_ratio() * car.differential[0].differentialFinalGearRatio,2); // DriveTrain Inertia powered wheels, I2=(wi1+wi2+ ... + wiN)/(gear ratio^2)
        
        float avgSpin = 0;
        for (int i = 0; i < car.poweredWheels.Length; i++) // Take the average wheelangularvelocity of all POWERED wheels
        {
            avgSpin += car.poweredWheels[i].wheelAngularVelocity / car.poweredWheels.Length;
        }

        w2 = avgSpin * (car.gearBox.get_ratio() * car.differential[0].differentialFinalGearRatio); // Drivetrain Angular Velocity

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
