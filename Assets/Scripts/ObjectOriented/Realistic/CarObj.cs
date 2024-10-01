using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Net.Sockets;
using UnityEngine;

public class CarObj : MonoBehaviour
{
    public enum SteeringLock {OFF, RIGHT, LEFT}
    [Header("References")]
    public EngineObj engine;
    public ClutchObj clutch;
    public GearBoxObj gearBox;
    public DifferentialObj differential;
    public WheelObj[] wheels;
    public WheelObj[] poweredWheels;
    private Rigidbody rb;

    [Header("BrakeInput Parameters")]
    public float maxBrakeTorque; // Maximum amount of braking possible for a car, Equals ebrake torque on ebrake wheels
    public float frontBrakeBias; // Where the brake torque is directed when using regular brakes, 1 = front bias, 0 = rear bias
    public bool hasABS; 

    [Header("Input")]
    public float Throttle;
    public bool ThrottleLock; // Auto accelerate
    public float BrakeInput;
    public float eBrakeInput;
    public float steeringInput;
    public float clampedSteeringAngle;
    public float maxSteeringAngle = 35f; // When the car is slow enough, this is as far as the wheels can turns
    public float minSteeringAngle = 15f; // When the car is fast enough, this is as far as the wheels can turns

    public SteeringLock steeringLock;
    public float k = 0.5f;



    [Header("Output")]
    public float Tc = 0; // Torque produced from the clutch after supplying it with the engine Torque
    public float torqueToWheel = 0; // Torque produced from the Gearbox after supplying it with the Torque CLutch
    // Start is called before the first frame update
    public float carSpeed = 0;
    public float avgFrontSlipAngle;
    public float avgBackSlipAngle;


    void Start()
    {
        rb = GetComponent<Rigidbody>();
        frontBrakeBias = Mathf.Clamp(frontBrakeBias, 0, 1); // Make sure it is within the correct boundaries
        
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
    }
    
    void FixedUpdate()
    {
        // Handle Input Here:
        GetInput(); 
        
        
        // Car Operation begins here:
        engine.engineOperation(); // Function for running the engine
		if(gearBox.get_ratio() != 0.0f) // If not in Neutral or shifting
			Tc = clutch.calculateClutch(); // Function for calculating clutch Torque (TC)
        torqueToWheel = Tc * gearBox.get_ratio() * differential.differentialFinalGearRatio / poweredWheels.Length; // Send TC into gearbox and differential, which becomes the torque to apply to wheels
        for (int i = 0; i < poweredWheels.Length/2; i++) // Iterate on each wheel axle that is powered by the engine
        {
            differential.calculateDifferential(i); // Each powered axle has a differential, we calculate how we spread the torque in accordance to the differential type here.
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
		    /*if (Input.GetKey(KeyCode.UpArrow))
            {
               Throttle = 1;
            }
			else
			{
				Throttle = 0;
			}*/
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
        if (Mathf.Abs(avgBackSlipAngle) > clampedSteeringAngle)
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
    #endregion

    
}
