using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Net.Sockets;
using UnityEngine;

public class CarObj : MonoBehaviour
{
    [Header("References")]
    public EngineObj engine;
    public ClutchObj clutch;
    public GearBoxObj gearBox;
    public DriveTrainObj driveTrain;
    public WheelObj[] wheels;
    public WheelObj[] poweredWheels;

    [Header("BrakeInput Parameters")]
    public float maxBrakeTorque;
    public float frontBrakeBias;

    [Header("Input")]
    public float Throttle;
    public bool ThrottleLock;
    public float BrakeInput;
    public float eBrakeInput;
    public float steeringInput;

    [Header("Output")]
    public float Tc = 0; // Torque produced from the clutch after supplying it with the engine Torque
    public float torqueToWheel = 0; // Torque produced from the Gearbox after supplying it with the Torque CLutch
    // Start is called before the first frame update

    void Start()
    {
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
        steeringInput = Input.GetAxis("Horizontal");
        
        // Car Operation begins here:
        engine.engineOperation(); // Function for running the engine
		if(gearBox.get_ratio() != 0.0f) // If not in Neutral or shifting
			Tc = clutch.calculateClutch(); // Function for calculating clutch Torque (TC)
        torqueToWheel = Tc * gearBox.get_ratio() * driveTrain.differentialFinalGearRatio / poweredWheels.Length; // Send TC into gearbox and differential, which becomes the torque to apply to wheels
        for (int i = 0; i < wheels.Length; i++)
        {
            wheels[i].brakeTorque = -Mathf.Sign(wheels[i].wheelAngularVelocity) * wheels[i].calculateBrakeTorque(BrakeInput, wheels[i].brakeBias, maxBrakeTorque);
            if (poweredWheels.Contains(wheels[i])) // We apply the torque to the powered wheels (Wheels that are directly driven by engine.)
            {
                wheels[i].applyTorqueToWheels(torqueToWheel); 
            }
            else
            {
                wheels[i].applyTorqueToWheels(0);
            }
			wheels[i].applyLongitudinalForce(); // Function for applying force based on Slip Ratio
            wheels[i].applyLateralForce();
        }


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
            
    }
    #endregion

    
}
