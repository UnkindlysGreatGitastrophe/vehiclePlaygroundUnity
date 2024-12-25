using System.Linq;
using Baracuda.Monitoring;
using Unity.IO.LowLevel.Unsafe;
using UnityEngine;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine.Animations;
using Palmmedia.ReportGenerator.Core.Reporting.Builders;

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
    [Header("Driving Assists")]

    public bool hasABS; // Toggles Anti Lock Brakes
    public bool hasTC;

    [Header("Input")]
    [Monitor] public float Throttle;
    public bool ThrottleLock; // Auto accelerate
    [Monitor] public float BrakeInput;
    public float eBrakeInput;
    public bool allowBarrelRoll;
    public bool PIDengaged;
    public float steeringInput;
    public float clampedSteeringAngle;
    public float maxSteeringAngle = 35f; // When the car is slow enough, this is as far as the wheels can turns
    public float minSteeringAngle = 15f; // When the car is fast enough, this is as far as the wheels can turns
    public SteeringLock steeringLock;
    public bool AutoReverseMode = false;


    [Header("DriveTrain Parameters")]
    public DriveType driveType = DriveType.RWD;

    public float torqueDistribution = 0.6f; // Option Applicable when AWD is selected, otherwise the power is distributed only to rear or front wheels.
    public float[] torqueRatio = new float[2];

    public float k = 0.5f; // Rate at which steering angle approaches min steering angle

    [Header("Aerial Dynamics")]

    public float downForce;
    public float airDensity;
    public float maxAero;
    public float aeroRate;
    public float airRotationAmount = 50f;
    public int numFrontFlips = 0;
     public int numBackFlips = 0;
     public int numLeftBarrelRolls = 0;
     public int numRightBarrelRolls = 0;
     public int numLeft360s = 0;
     public int numRight360s = 0;
     public Vector3 totalRotation = Vector3.zero;
     public Quaternion lastRotation;
     private Vector3 angleDiffs;

    bool originInit = false;

    [Header("PID")]
    public float proportionalGain = 10f;
    public float integralGain = 2f;
    public float derivativeGain = 2f;
    Vector3 errorLast = Vector3.zero;
    Vector3 integrationStored = Vector3.zero;

    RaycastHit RaycastDir;
    bool isHit;


    [Header("Output")]
    public float Tc = 0; // Torque produced from the clutch after supplying it with the engine Torque
    [Monitor] public float torqueToWheel = 0; // Torque produced from the Gearbox after supplying it with the Torque CLutch
    // Start is called before the first frame update
    [Monitor] public float carSpeed = 0;
    public float avgFrontSlipAngle;
    public float avgBackSlipAngle;
    public float avgBackSlipRatio;




    #region Start/Update
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
		if(gearBox.get_ratio() != 0.0f && gearBox.gearEngaged) // If not in Neutral or shifting
			Tc = clutch.calculateClutch(); // Function for calculating clutch Torque (TC)
        else
        {
            Tc = 0;
            engine.clutch_torque = 0;
            clutch.clutchLock = 0;
        }
            
        //torqueToWheel = Tc * gearBox.get_ratio() * differential[0].differentialFinalGearRatio / poweredWheels.Length; // Send TC into gearbox and differential, which becomes the torque to apply to wheels
        for (int i = 0; i < differential.Length; i++) // Iterate on each wheel axle that is powered by the engine
        {
            torqueToWheel = (Tc * gearBox.get_ratio() * gearBox.finalDriveGear) * torqueRatio[i];
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

        if (isCarMidAir())
        {
            if (!originInit)
            {
                originInit = true;
                lastRotation = rb.rotation;
            }
            if (PIDengaged)
            {
                PID();
            }
            AerialRotation();
            TrackStunts();
            
        }
        else
        {
            originInit = false;
        }
        carSpeed = transform.InverseTransformDirection(rb.velocity).z * 3.6f;


    }

    #endregion

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
                allowBarrelRoll = true;
            }
            else
            {
                eBrakeInput = 0;
                allowBarrelRoll = false;
            }
            if (Input.GetKey(KeyCode.LeftControl))
            {
                PIDengaged = true;
            }
            else
            {
                PIDengaged = false;
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
                if (gearBox.Transmissiontype == GearBoxObj.GearboxType.MANUAL)
                {
                    if (Input.GetAxisRaw("Vertical") == 1 && gearBox.gearEngaged == true)
                    {
                        Throttle = Mathf.Clamp(Throttle + Time.fixedDeltaTime * 1, 0, 1);
                    }
                    else if (gearBox.gearEngaged == true)
                    {
                        Throttle = Mathf.Clamp(Throttle + Time.fixedDeltaTime * -3, 0, 1);
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
                if (gearBox.Transmissiontype == GearBoxObj.GearboxType.AUTOMATIC)
                {
                    if (!AutoReverseMode)
                    {
                        if (Input.GetAxisRaw("Vertical") == 1 && gearBox.gearEngaged == true)
                        {
                            Throttle = Mathf.Clamp(Throttle + Time.fixedDeltaTime * 1, 0, 1);
                        }
                        else if (gearBox.gearEngaged == true)
                        {
                            Throttle = Mathf.Clamp(Throttle + Time.fixedDeltaTime * -3, 0, 1);
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
                    else
                    {
                        if (Input.GetAxisRaw("Vertical") == 1 && gearBox.gearEngaged == true)
                        {
                            BrakeInput = Mathf.Lerp(BrakeInput, 1, 8*Time.deltaTime);
                        }
                        else if (gearBox.gearEngaged == true)
                        {
                            BrakeInput = Mathf.Lerp(BrakeInput, 0, 16*Time.deltaTime);
                        }

                        if (Input.GetAxisRaw("Vertical") == -1)
                        {
                            Throttle = Mathf.Clamp(Throttle + Time.fixedDeltaTime * 1, 0, 1);
                        }
                        else
                        {
                            Throttle = Mathf.Clamp(Throttle + Time.fixedDeltaTime * -3, 0, 1);
                        }

                    }
                    
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


    public float GetAVGWheelRPM()
    {
        float AVG = 0;
        for (int i = 0; i < wheels.Length; i++)
        {
            AVG += wheels[i].wheelAngularVelocity * engine.AV_2_RPM / wheels.Length;
        }
        return AVG;
    }

    #endregion

    #region Aerial Dynamics

    bool isCarMidAir()
    // Check to see if all 4 tires are off the ground
    {
        for (int w = 0; w < wheels.Length; w++)
        {
            if (wheels[w].isHit == true)
            {
                return false;
            }
        }
        return true;
    }
    void GetDownForce()
    {
        downForce = Mathf.Min(maxAero,0.5f*airDensity * Mathf.Pow(Mathf.Abs(carSpeed),2)*aeroRate);
        //rb.AddForceAtPosition(Vector3.down*downForce, rb.centerOfMass);
    }

    void AerialRotation()
    {


        // Allow the car to Rotate
        float h = Input.GetAxis("Horizontal") * airRotationAmount * Time.deltaTime;
        float v = Input.GetAxis("Vertical") * airRotationAmount * Time.deltaTime;

        rb.AddTorque(rb.transform.right * v, ForceMode.Acceleration);
        if (allowBarrelRoll)
        {
            rb.AddRelativeTorque(0f, 0f, h, ForceMode.Acceleration);
        }
        else
        {
            rb.AddTorque(rb.transform.up * h, ForceMode.Acceleration);
        }
        //Debug.DrawRay(rb.position, new Vector3(rb.velocity.x,Physics.gravity.y, rb.velocity.z), Color.cyan);
    }

    void TrackStunts()
    {
        angleDiffs = Vector3.zero;
        Quaternion difference = Quaternion.Inverse(rb.rotation) * lastRotation;
        lastRotation = rb.rotation;

        angleDiffs.x = Mathf.DeltaAngle(0, difference.eulerAngles.x);
        angleDiffs.y = Mathf.DeltaAngle(0, difference.eulerAngles.y);
        angleDiffs.z = Mathf.DeltaAngle(0, difference.eulerAngles.z);

        totalRotation.x += angleDiffs.x;

        if (totalRotation.x < -360f)
        {
            totalRotation.x = 0;
            numFrontFlips++;
        }
        else if (totalRotation.x > 360f)
        {
            totalRotation.x = 0;
            numBackFlips++;
        }

        totalRotation.y += angleDiffs.y;

        if (totalRotation.y < -360f)
        {
            totalRotation.y = 0;
            numRight360s++;
        }
        else if (totalRotation.y > 360f)
        {
            totalRotation.y = 0;
            numLeft360s++;
        }

        totalRotation.z += angleDiffs.z;

        if (totalRotation.z < -360f)
        {
            totalRotation.z = 0;
            numRightBarrelRolls++;
        }
        else if (totalRotation.z > 360f)
        {
            totalRotation.z = 0;
            numLeftBarrelRolls++;
        }
    }

    void PID()
    {
        
        if (Mathf.Abs(rb.transform.localRotation.eulerAngles.z) > 60f && Mathf.Abs(carSpeed) <= 5f) {
        rb.AddRelativeTorque(0f, 0f, 30, ForceMode.Acceleration);
        }

        bool isHit = Physics.Raycast(rb.position, new Vector3(rb.velocity.x,Physics.gravity.y, rb.velocity.z), out RaycastDir); // Raycast

        Debug.DrawRay(rb.position, new Vector3(rb.velocity.x,Physics.gravity.y, rb.velocity.z), Color.cyan);


        if (isHit)
        {

            Debug.DrawRay(RaycastDir.point, RaycastDir.normal, Color.magenta);
            Debug.DrawRay(rb.position, rb.transform.up, Color.magenta);


            

            Vector3 targetValue = RaycastDir.normal;
            //Vector3 error = targetValue - rb.transform.up;
            Vector3 error = Vector3.Cross(rb.transform.up, targetValue);
            // Calculate P
            Vector3 P = proportionalGain * error;
            // Calculate D term
            Vector3 errorRateOfChange = (error - errorLast) / Time.fixedDeltaTime;
            errorLast = error;


            Vector3 D = derivativeGain * errorRateOfChange;
            

                        // Calculate I term
            integrationStored = integrationStored + (error * Time.fixedDeltaTime);
            Vector3 I =  integrationStored * integralGain;


            rb.AddTorque(P+D+I, ForceMode.Impulse);
        }


        // float error = targetValue - currentValue;

    }

    #endregion

    
}
