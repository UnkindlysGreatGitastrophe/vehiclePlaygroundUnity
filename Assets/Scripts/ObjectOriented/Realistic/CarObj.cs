using System.Linq;
using Baracuda.Monitoring;
using Unity.IO.LowLevel.Unsafe;
using UnityEngine;
using System.Collections.Generic;

public class CarObj : MonoBehaviour
{
    public enum SteeringLock {OFF, RIGHT, LEFT}
    public enum DriveType {FWD, RWD, AWD};

    [Header("References")]
    public EngineObj engine;
    public ClutchObj clutch;
    public GearBoxObj gearBox;
    public DifferentialObj[] differential;
    public WheelObj[] wheels;
    public WheelObj[] poweredWheels;
    private Rigidbody rb;

    [Header("BrakeInput Parameters")]
    public float maxBrakeTorque; // Maximum amount of braking possible for a car, Equals ebrake torque on ebrake wheels
    public float frontBrakeBias; // Where the brake torque is directed when using regular brakes, 1 = front bias, 0 = rear bias
    public bool hasABS; 
    public bool hasTC;

    [Header("Input")]
    [Monitor] public float Throttle;
    public bool ThrottleLock; // Auto accelerate
    public float BrakeInput;
    public float eBrakeInput;
    public float steeringInput;
    public float clampedSteeringAngle;
    public float maxSteeringAngle = 35f; // When the car is slow enough, this is as far as the wheels can turns
    public float minSteeringAngle = 15f; // When the car is fast enough, this is as far as the wheels can turns
    public SteeringLock steeringLock;


    [Header("DriveTrain Parameters")]
    public DriveType driveType = DriveType.RWD;

    public float torqueDistribution = 0.6f; // Option Applicable when AWD is selected, otherwise the power is distributed only to rear or front wheels.
    public float[] torqueRatio = new float[2];

    public float k = 0.5f; // Rate at which steering angle approaches min steering angle

    [Header("Aerodynamics")]
    public float downForce;
    public float airDensity;
    public float maxAero;
    public float aeroRate;


    [Header("Output")]
    public float Tc = 0; // Torque produced from the clutch after supplying it with the engine Torque
    [Monitor] public float torqueToWheel = 0; // Torque produced from the Gearbox after supplying it with the Torque CLutch
    // Start is called before the first frame update
    [Monitor] public float carSpeed = 0;
    public float avgFrontSlipAngle;
    public float avgBackSlipAngle;
    [Monitor] public float avgBackSlipRatio;


    void Start()
    {
        this.StartMonitoring();
        rb = GetComponent<Rigidbody>();
        frontBrakeBias = Mathf.Clamp(frontBrakeBias, 0, 1); // Make sure it is within the correct boundaries
        torqueDistribution = Mathf.Clamp(torqueDistribution, 0, 1); // Make sure it is within the correct boundaries
        torqueRatio[0] = torqueDistribution;
        torqueRatio[1] = 1-torqueDistribution;
        differential = transform.GetComponentsInChildren<DifferentialObj>();

        for (int w = 0; w < wheels.Length; w++) // Set up brake bias here, if past the halfway mark of the wheels, then we need to assign the rear brake bias instead of the front
        {
            if (w < wheels.Length / 2)
            {
                wheels[w].brakeBias = frontBrakeBias;
            }
            else
            {
                wheels[w].brakeBias = 1 - frontBrakeBias;
            }
        }

        if (driveType == DriveType.AWD)
        {
            poweredWheels = wheels;

        }
        else if (driveType == DriveType.RWD)
        {
            int idx = 0;
            poweredWheels = new WheelObj[2];
            for (int w = 0; w < wheels.Length; w++)
            {
                if (wheels[w].name.StartsWith("R"))
                {
                    poweredWheels[idx++] = wheels[w];
                }
            }
            torqueRatio[0] = 1;
            torqueRatio[1] = 0;

        }
        else {
            int idx = 0;
            poweredWheels = new WheelObj[2];
            for (int w = 0; w < wheels.Length; w++)
            {
                if (wheels[w].name.StartsWith("F"))
                {
                    poweredWheels[idx++] = wheels[w];
                }
            }
            torqueRatio[0] = 0;
            torqueRatio[1] = 1;
        }
    }
    
    void FixedUpdate()
    {
        // Handle Input Here:
        GetInput(); 
        
        GetDownForce();
        // Car Operation begins here:
        engine.engineOperation(); // Function for running the engine
		if(gearBox.get_ratio() != 0.0f) // If not in Neutral or shifting
			Tc = clutch.calculateClutch(); // Function for calculating clutch Torque (TC)
        else
        {
            Tc = 0;
            engine.clutch_torque = 0;
        }
            
        //torqueToWheel = Tc * gearBox.get_ratio() * differential[0].differentialFinalGearRatio / poweredWheels.Length; // Send TC into gearbox and differential, which becomes the torque to apply to wheels
        for (int i = 0; i < differential.Length; i++) // Iterate on each wheel axle that is powered by the engine
        {
            torqueToWheel = (Tc * gearBox.get_ratio() * differential[0].differentialFinalGearRatio) * torqueRatio[i];
            differential[i].calculateDifferential(); // Each powered axle has a differential, we calculate how we spread the torque in accordance to the differential type here.
            //Debug.Log(poweredWheels[i].wheelAngularVelocity - poweredWheels[i+1].wheelAngularVelocity); // Debug
        }
        for (int i = 0; i < wheels.Length; i++) // Iterate on all whels
        {
            wheels[i].brakeTorque = -Mathf.Sign(wheels[i].wheelAngularVelocity) * wheels[i].calculateBrakeTorque(BrakeInput, wheels[i].brakeBias, maxBrakeTorque); // Get the brake torque here
            if (!poweredWheels.Contains(wheels[i])) // If the wheel is not powered at all, apply no drive torque here.
                wheels[i].applyTorqueToWheels(0);
			wheels[i].calculateLongitudinalForce(); // Function for calculating longitudinal force based on Slip Ratio
            wheels[i].calculateLateralForce(); // Function for calculating lateral force based on Slip Angle
            wheels[i].applyWheelForces(); // Function for applying a combination of Lateral and longitudinal force
        }
        carSpeed = transform.InverseTransformDirection(rb.velocity).z * 3.6f;


    }

    #region Input Handling
    public void GetInput()
    {
		    if (Input.GetKey(KeyCode.Z))
            {
               Time.timeScale = 1;
            }
			if (Input.GetKey(KeyCode.X))
            {
               Time.timeScale = 0.5f;
            }
            if (Input.GetKey(KeyCode.C))
            {
               Time.timeScale = 0.1f;
            }
            if (Input.GetKey(KeyCode.V))
            {
               Time.timeScale = 0.01f;
            }
            if (Input.GetKey(KeyCode.Space))
            {
                eBrakeInput = 1;
            }
            else
            {
                eBrakeInput = 0;
            }
            if (Input.GetKeyDown(KeyCode.T))
            {
                ThrottleLock = !ThrottleLock;
            }
            if (ThrottleLock)
            {
                Throttle = 1;
            }
            else
            {
                if (Input.GetAxisRaw("Vertical") == 1)
                {
                    Throttle = Mathf.Clamp(Throttle + Time.fixedDeltaTime * 4, 0, 1);
                }
                else
                {
                    Throttle = Mathf.Clamp(Throttle + Time.fixedDeltaTime * -10, 0, 1);
                }

                if (Input.GetAxisRaw("Vertical") == -1)
                {
                    BrakeInput = Mathf.Lerp(BrakeInput, 1, 8*Time.deltaTime);
                }
                else
                {
                    BrakeInput = Mathf.Lerp(BrakeInput, 0, 16*Time.deltaTime);
                }
            }

            switch(steeringLock) 
            {
            case SteeringLock.LEFT:
                steeringInput = -1;
                break;
            case SteeringLock.RIGHT:
                steeringInput = 1;
                break;
            default:
                steeringInput = Input.GetAxis("Horizontal");
                GetSpeedBasedSteerAngle();
                //GetSlipBasedSteerAngle();
                break;
            }
            
    }

    void GetSpeedBasedSteerAngle()
    {
        clampedSteeringAngle = Mathf.Clamp(maxSteeringAngle *  (1.0f / (1.0f + Mathf.Abs(carSpeed) * k)),minSteeringAngle,maxSteeringAngle);
        avgBackSlipAngle = 0;
        for (int i = 2; i < wheels.Length; i++)
        {
            avgBackSlipAngle += wheels[i].slipAngle * Mathf.Rad2Deg / 2;
        }
        if (Mathf.Abs(avgBackSlipAngle) > clampedSteeringAngle + 5)
        {
            clampedSteeringAngle = Mathf.Clamp(Mathf.Abs(avgBackSlipAngle),minSteeringAngle,maxSteeringAngle);
        }
    }


    void GetSlipBasedSteerAngle()
    {
        avgFrontSlipAngle = 0;
        avgBackSlipAngle = 0;

        for (int i = 0; i < 2; i++)
        {
            avgFrontSlipAngle += wheels[i].slipAngle * Mathf.Rad2Deg / 2;
        }
        clampedSteeringAngle = Mathf.Clamp(avgFrontSlipAngle,minSteeringAngle,maxSteeringAngle);
        for (int i = 2; i < wheels.Length; i++)
        {
            avgBackSlipAngle += wheels[i].slipAngle * Mathf.Rad2Deg / 2;
        }
        if (Mathf.Abs(avgBackSlipAngle) > clampedSteeringAngle)
        {
            clampedSteeringAngle = Mathf.Clamp(Mathf.Abs(avgBackSlipAngle),minSteeringAngle,maxSteeringAngle); // Assist with spinning out
        }

        
    }

    void GetDownForce()
    {
        downForce = Mathf.Min(maxAero,0.5f*airDensity * Mathf.Pow(Mathf.Abs(carSpeed),2)*aeroRate);
        //rb.AddForceAtPosition(Vector3.down*downForce, rb.centerOfMass);
    }
    #endregion

    
}
