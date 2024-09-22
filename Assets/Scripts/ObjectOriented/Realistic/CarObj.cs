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
    public float maxBrakeTorque;
    public float frontBrakeBias;

    [Header("Input")]
    public float Throttle;
    public bool ThrottleLock;
    public float BrakeInput;
    public float eBrakeInput;
    public float steeringInput;
    public float clampedSteeringAngle;
    public float maxSteeringAngle = 35f; 

    public SteeringLock steeringLock;
    public float k = 0.5f;



    [Header("Output")]
    public float Tc = 0; // Torque produced from the clutch after supplying it with the engine Torque
    public float torqueToWheel = 0; // Torque produced from the Gearbox after supplying it with the Torque CLutch
    // Start is called before the first frame update
    public float carSpeed = 0;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        frontBrakeBias = Mathf.Clamp(frontBrakeBias, 0, 1);
        for (int w = 0; w < wheels.Length; w++)
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
        for (int i = 0; i < poweredWheels.Length/2; i++)
        {
            float torqueOffset = differential.calculateDifferential(i);
            poweredWheels[i].applyTorqueToWheels(torqueToWheel+torqueOffset);
            poweredWheels[i+1].applyTorqueToWheels(torqueToWheel-torqueOffset);
            Debug.Log(poweredWheels[i].wheelAngularVelocity - poweredWheels[i+1].wheelAngularVelocity);
        }
        for (int i = 0; i < wheels.Length; i++)
        {
            wheels[i].brakeTorque = -Mathf.Sign(wheels[i].wheelAngularVelocity) * wheels[i].calculateBrakeTorque(BrakeInput, wheels[i].brakeBias, maxBrakeTorque);
            if (!poweredWheels.Contains(wheels[i]))
                wheels[i].applyTorqueToWheels(0);
			wheels[i].applyLongitudinalForce(); // Function for applying force based on Slip Ratio
            wheels[i].applyLateralForce();
        }
        carSpeed = rb.velocity.z * 3.6f;


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
                clampedSteeringAngle = maxSteeringAngle *  (1.0f / (1.0f + Mathf.Abs(carSpeed) * k));
                break;
            }
            
    }
    #endregion

    
}
