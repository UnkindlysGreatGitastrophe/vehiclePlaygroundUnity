using System.Linq;
using Baracuda.Monitoring;
using UnityEngine;
using Unity.VisualScripting;
using System.Text.RegularExpressions;
using System.Collections;


public class CarObj : MonoBehaviour
{
    public enum SteeringLock {OFF, RIGHT, LEFT}
    public enum DriveType {FWD, RWD, AWD};

    [Header("References")]
    public EngineObj engine;
    public ForcedInductionObj induction;
    public ClutchObj clutch;
    public GearBoxObj gearBox;
    public DifferentialObj[] differential;
    public WheelObj[] wheels;
    public WheelObj[] poweredWheels;
    private Rigidbody rb;

    [Header("Brake Input Parameters")]

    [Tooltip("Maximum amount of braking possible for a car, This is the same force that will be applied on e-brake compatible wheels")]
    public float maxBrakeTorque; // Maximum amount of braking possible for a car, Equals ebrake torque on ebrake wheels
    [Tooltip("Directs the distribution of brake force to the front and back of the wheels, 1 = front brakes only, 0 = rear brakes only")]
    public float frontBrakeBias; // Where the brake torque is directed when using regular brakes, 1 = front bias, 0 = rear bias
    
    [Header("Driving Assists")]

    [Tooltip("Toggles Anti-Lock Brakes, efficient braking with minimal lockup")]
    public bool hasABS; // Toggles Anti Lock Brakes
    [Tooltip("Toggles Traction Control, limits excess wheelspin")]
    public bool hasTC; // Toggles Traction Control

    [Header("Input")]

    [Tooltip("The gas pedal!, 0 = no throttle, 1 = full throttle")]
    [Monitor] public float Throttle; // 0-1, 0 is no throttle, 1 is full throttle

    [Tooltip("The Brake pedal, 0 = no brakes, 1 = full brakes")]
    [Monitor] public float BrakeInput; // 0-1, 0 is no brakes, 1 is full brakes
    [Tooltip("The Spicy brake pedal, 0 = no handbrake, 1 = full handbrake")]
    public float eBrakeInput; // 0-1, 0 is no handbrake, 1 is full hand brake
    [Tooltip("Toggles the ability to perform barrel rolls instead of flat spins")]
    public bool allowBarrelRoll; // Used to allow the car to do barrel rolls through Input(Horizontal)
    [Tooltip("Toggles the ability to use PID stabilization to counter body rolling in jumps")]
    [Monitor] public bool PIDengaged; // Bool that allows the car to stablize itself in the air
    [Tooltip("The direction of which the car turns is determined by this variable, Range is -1 <= steeringInput <= 1, (-1 is left, 0 is straight, 1 is right)")]
    public float steeringInput; // -1 to 1, left is -1, 0 is straight, 1 is right
    [Tooltip("The maximum steering angle a car can do in a given situation, inversely affected by top speed for smooth driving at speeds")]
    public float clampedSteeringAngle; // The maximum steering angle a car can do when under speed or spinning out\
    [Tooltip("The maximum steering angle a car can possibly do")]
    public float maxSteeringAngle = 35f; // When the car is slow enough, or spinning out, this is as far as the wheels can turns
    [Tooltip("The minimum steering angle a car can possibly do")]
    public float minSteeringAngle = 15f; // When the car is fast enough, this is as far as the wheels can turns
    [Tooltip("Rate of which the steering angle approaches minimum steering angle")]
    public float k = 0.5f; // Rate at which steering angle approaches min steering angle
    [Tooltip("When true, the gas and brake input is inverted to allow smoother gameplay when using automatic transmissions (Does NOT apply to manuals)")]
    public bool AutoReverseMode = false; // Reverse mode that inverts input when activated (Automatic Gearbox only)

    [Tooltip("DEBUG: Toggles full steering lock")]
    public SteeringLock steeringLock; // Debugging
    [Tooltip("DEBUG: Toggles auto acceleration, will break shifting by keeping throttle on")]
    public bool ThrottleLock; // Auto accelerate


    [Header("DriveTrain Parameters")]
    [Tooltip("Determines Drivetrain type (RWD, AWD, FWD)")]
    public DriveType driveType = DriveType.RWD; // Determines Drivetrain type (RWD, AWD, FWD?)
    [Tooltip("Distributes torque to the front and rear wheel axles, approach 1 for a rear wheel bias, approach 0 for a front wheel bias (NOTE: this variable is only relevant when the driveType is AWD)")]
    public float torqueDistribution = 0.6f; // Option Applicable when AWD is selected, otherwise the power is distributed only to rear or front wheels.
    [Tooltip("Ratio of torque distribution, first element is for the rear wheels, 2nd element is for the front wheels")]
    public float[] torqueRatio = new float[2]; // Ratio of torque distribution


    [Header("AeroDynamics")]
    [Tooltip("A downward force that pushes the car into the ground for extra traction, increases with speed")]
    public float downForce; // Downforce spread across the car's wheels
    [Tooltip("A constant, similiar to real life calculations")]
    public float airDensity; // Constant
    [Tooltip("The maximum possible downforce achieveable")]
    public float maxAero; // Maximum Aero Downforce possible
    [Tooltip("Rate of which downforce is gained with speed")]
    public float aeroRate; // Rate of which Downforce is gained with speed
    [Header("Stunts")]
    [Tooltip("Rate of which the car can rotate in the air")]
    public float airRotationAmount = 50f; // Rate of which the rotation of the car is performed when in the air
    [Tooltip("Stunts!")]
    public int numFrontFlips = 0; // Stunt Count
    [Tooltip("Stunts!")]
    public int numBackFlips = 0;
    [Tooltip("Stunts!")]
    public int numLeftBarrelRolls = 0;
    [Tooltip("Stunts!")]
    public int numRightBarrelRolls = 0;
    [Tooltip("Stunts!")]
    public int numLeft360s = 0;
    [Tooltip("Stunts!")]
    public int numRight360s = 0;
    [Tooltip("Total amount of rotation made in a particular axis relative to an original rotation")]

    public Vector3 totalRotation = Vector3.zero; // Total rotation performed in the car relative to a lastRotation
    [Tooltip("The pivot point of which total rotation is calclated off of")]
    public Quaternion lastRotation; // The pivot point of which total rotation is calclated off of
    [Tooltip("The Euler representation of the difference between the current and last rotation")]
    private Vector3 angleDiffs; // The difference between the current and lastRotation
    [Tooltip("When true, the last rotation prior to taking a jump will be stored for tracking stunts")]
    bool originInit = false; // Bool that initializes the lastRotation the moment we catch air

    [Header("PID")]
    [Tooltip("How quick the car rotation approaches the target rotation")]
    public float proportionalGain = 10f; // How quick the rotation approaches the target
    [Tooltip("How exact we want the rotation to be at the target (Preferable to set this relatively low to the other gain variables)")]
    public float integralGain = 2f; // How exact we want the rotation to be at the target
    [Tooltip("How quickly the car rotation slows as it reaches target rotation")]
    public float derivativeGain = 2f; // How quickly the rotiation slows as it reaches target
    [Tooltip("Stores the last error for derivative gain calculation")]
    Vector3 errorLast = Vector3.zero;
    [Tooltip("Stores the last integration for integral gain calculation")]
    Vector3 integrationStored = Vector3.zero;
    RaycastHit RaycastDir; // Data to Store RayCast collisions

    [Header("Nitro Properties")]

    [Tooltip("The time it takes to be able to reactivate nitro once the player lays off the boost")]
    [Monitor] public float nitroDelay = 0f; // The time it takes to be able to reactivate nitro once the player lays off the boost
    public float nitroDelayTime = 0.4f; // In seconds, how much the player will need to wait until they are able to use nitro again
    public float nitroOverBoostDelayTime = 1.5f; // In seconds, how much the player will need to wait until they are able to use nitro again after overboosting
    public float nitroHeatRate = 1.5f; // The rate of which the heating of the nitro increases as the nitro is used
    public float nitroCoolRate = 1; // The rate of which the heating of the nitro decreases as the nitro is disengaged
    public float airNitroCoolRate = 2; // The rate of which the heating of the nitro decreases as the nitro is disengaged while in mid-air
    public float maxOverBoostPenalty = 3; // In seconds, this determines the amount of time a car can spend overboosting before it explodes
    [Monitor] public float nitroValue = 0; // From a range of 0 (No nitro use) to 1 (Overheating), this variable determines how hot the nitro temperature is
    [Monitor] private float nitroOverBoostValue = 0; // From a range of 0 (Not overboosting) to the maxOverBoostPenalty (Car explodes from overboosting), this variable determines how much overboosting a player is doing
    [Monitor] public bool isOverBoosting; // Determines if the car is in the overboost zone
    [Monitor] private bool nitroDelayInit; // Used to initialize delays after the player stops using nitro
    [Monitor] public bool nitroOn; // Indicates if the nitro is being used or not

    public float kineticBoost = 1500f;
    public float kineticBoostMultiplier = 3.5f;
    public float nitrousPower = 0.5f;

    


    [Header("Output")]
    [Tooltip("Clutch Torque that holds the clutch plates together, and keep them spinning at the same rate as much as possible")]
    public float Tc = 0; // Clutch Torque, produced from balancing the engine speed and the transmission speed together, if the speeds are too much, then the clutch is disconnected, and the Clutch Torque is 0
    [Tooltip("Torque produced from a forced induction system")] 
    public float inductionTorque;
    [Tooltip("Torque produced from the Gearbox ratio and final drive gear that gets sent to each axle")]
    public float torqueToAxle = 0; // Torque produced from the Gearbox after supplying it with the Torque CLutch
    // Start is called before the first frame update
    [Tooltip("Measured in KM/H")]
    [Monitor] public float carSpeed = 0; // Measured in KM/H
    [Tooltip("Average Slip Angle of rear wheels")]
    public float avgBackSlipAngle; // Average Back Slip Angles
    [Tooltip("Average Slip Ratio of rear wheels")]
    public float avgBackSlipRatio; // Average Back Slip Ratio






    #region Start
    void Start()
    {
        // DEBUGGING
        this.StartMonitoring();

        // REFERENCING
        rb = GetComponent<Rigidbody>(); // Get the RigidBody Component of car
        differential = transform.GetComponentsInChildren<DifferentialObj>(); // Get Differential

        // VARIABLE INITIALIZING
        frontBrakeBias = Mathf.Clamp(frontBrakeBias, 0, 1); // Make sure it is within the correct boundaries
        torqueDistribution = Mathf.Clamp(torqueDistribution, 0, 1); // Make sure it is within the correct boundaries
        torqueRatio[0] = torqueDistribution; // Distribute power to rear wheels
        torqueRatio[1] = 1-torqueDistribution; // Then the remainder to the front wheels

        InitializePoweredWheels();

        
    }


    void InitializePoweredWheels()
    {
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

        if (driveType == DriveType.AWD) // Make all wheels powered
        {
            poweredWheels = wheels;
        }
        else if (driveType == DriveType.RWD) // Make only the Rear wheels powered (REAR WHEELS MUST START WITH R)
        {
            int idx = 0;
            poweredWheels = new WheelObj[2]; // Create list here
            for (int w = 0; w < wheels.Length; w++)
            {
                if (wheels[w].name.StartsWith("R"))
                {
                    poweredWheels[idx++] = wheels[w]; // Add in any wheel that starts with R (Rear)
                }
            }
            torqueRatio[0] = 1; // Update Torque Ratio accordingly
            torqueRatio[1] = 0;
        }
        else { // Make only Front wheels powered (FWD)
            int idx = 0;
            poweredWheels = new WheelObj[2];
            for (int w = 0; w < wheels.Length; w++)
            {
                if (wheels[w].name.StartsWith("F"))
                {
                    poweredWheels[idx++] = wheels[w]; // Similiar process for RWD, but for Front wheels only (F), and opposite torque ratio
                }
            }
            torqueRatio[0] = 0;
            torqueRatio[1] = 1;
        }
    }

    #endregion

    #region Update
    
    /*void OnDrawGizmos() // Used to debug center of gravity
    {
        if (rb != null)
        {
        Gizmos.color = Color.magenta;
        Gizmos.DrawSphere(transform.position + transform.rotation * rb.centerOfMass,1f);
        }
    }
    */

    void FixedUpdate()
    {
        // Handle Input Here:
        GetInput(); 
        
        GetDownForce();
        // Car Operation begins here:
        engine.engineOperation(); // Function for running the engine
        if (induction != null)
        {
            if (induction.type == ForcedInductionObj.ForcedInductionType.TURBO)
                induction.calculateTurboPSI();
            else if (induction.type == ForcedInductionObj.ForcedInductionType.SUPERCHARGE)
                induction.calculateSuperChargePSI();
            inductionTorque = induction.convertPSItoTorque();
            engine.torque_out += inductionTorque;
        }
		if(gearBox.get_ratio() != 0.0f && gearBox.gearEngaged) // If not in Neutral or shifting
			Tc = clutch.calculateClutch(); // Function for calculating clutch Torque (TC)
        else
        {
            Tc = 0; // Clutch is not connected
            engine.clutch_torque = 0;
            clutch.clutchLock = 0;
        }
            
        for (int i = 0; i < differential.Length; i++) // Iterate on each wheel axle that is powered by the engine
        {
            torqueToAxle = (Tc * gearBox.get_ratio() * gearBox.finalDriveGear) * torqueRatio[i]; // Calculate Potential Torque to each wheel
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

        if (isCarMidAir()) // If the car is in the air, All 4 raycasts are not touching the ground, we allow the mid-air manuevers
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
            PIDengaged = true;
        }
        carSpeed = transform.InverseTransformDirection(rb.velocity).z * 3.6f; // Measured in KM/H
        


    }

    #endregion

    #region Input Handling
    public void GetInput() 
    {       
        // DEBUGGING

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

        if (Input.GetKeyDown(KeyCode.T))
        {
            ThrottleLock = !ThrottleLock; // Note that throttle lock set to true brakes throttle when shifting
        }
        if (ThrottleLock)
        {
            Throttle = 1;
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
            break;
        }

        // END DEBUGGING CODE

        // Input that handles E-Brake
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

        if (Input.GetKey(KeyCode.LeftShift) && nitroDelay == 0)
        {
            nitroOn = true;
            nitroDelayInit = true;
            activateNitro();
        }
        else
        {
            nitroOn = false;
            nitroValue = Mathf.Clamp(nitroValue - Time.fixedDeltaTime * nitroCoolRate, 0, 1);
            if (nitroDelayInit)
            {
                nitroDelay = (isOverBoosting) ? nitroOverBoostDelayTime : nitroDelayTime;
                nitroDelayInit = false;
            }
            isOverBoosting = false;
            engine.nitroTorque = 0;
            StartCoroutine(NitroReactivation());

        }

        /*// Input that handles PID Stabilization
        if (Input.GetKey(KeyCode.LeftControl))
        {
            PIDengaged = true;
        }
        */

        if (!ThrottleLock) // If Throttle is not locked, then operate the input normally
        {
            if (gearBox.Transmissiontype == GearBoxObj.GearboxType.MANUAL) // Manual Transmissions operate as normal
        {
            if (Input.GetAxisRaw("Vertical") == 1 && gearBox.gearEngaged == true) // Acceleration of engine, also assumes gearbox is engaged, otherwise we let the gearbox script handle throttle when shifting gears
            {
                Throttle = Mathf.Clamp(Throttle + Time.fixedDeltaTime * 1, 0, 1);
                if (isCarMidAir() && Input.GetKey(KeyCode.Space))
                {
                    PIDengaged = false;
                }
            }
            else if (gearBox.gearEngaged == true)
            {
                Throttle = Mathf.Clamp(Throttle + Time.fixedDeltaTime * -3, 0, 1);
            }

            if (Input.GetAxisRaw("Vertical") == -1) // Brakes, operate independently of gearbox
            {
                BrakeInput = Mathf.Lerp(BrakeInput, 1, 8*Time.deltaTime);
                if (isCarMidAir() && Input.GetKey(KeyCode.Space))
                {
                    PIDengaged = false;
                }
            }
            else
            {
                BrakeInput = Mathf.Lerp(BrakeInput, 0, 16*Time.deltaTime);
            }
        }
        if (gearBox.Transmissiontype == GearBoxObj.GearboxType.AUTOMATIC) // Automatic transmission handles extra cases, brake and throttle inverts when driver wants to reverse.
        {
            if (!AutoReverseMode)
            {
                if (Input.GetAxisRaw("Vertical") == 1 && gearBox.gearEngaged == true) // Operate as normal if we aren't in reverse mode for automatics
                {
                    Throttle = Mathf.Clamp(Throttle + Time.fixedDeltaTime * 1, 0, 1);
                    if (isCarMidAir() && Input.GetKey(KeyCode.Space))
                    {
                        PIDengaged = false;
                    }
                }
                else if (gearBox.gearEngaged == true)
                {
                    Throttle = Mathf.Clamp(Throttle + Time.fixedDeltaTime * -3, 0, 1);
                }

                if (Input.GetAxisRaw("Vertical") == -1)
                {
                    BrakeInput = Mathf.Lerp(BrakeInput, 1, 8*Time.deltaTime);   // Otherwise, swap the controls so that gas is brakes
                    if (isCarMidAir() && Input.GetKey(KeyCode.Space))
                    {
                        PIDengaged = false;
                    }   
                }
                else
                {
                    BrakeInput = Mathf.Lerp(BrakeInput, 0, 16*Time.deltaTime);
                }
            }
            else
            {
                if (Input.GetAxisRaw("Vertical") == 1 && gearBox.gearEngaged == true) // Similiar to how throttle button is handled but is vice versa for brakes
                {
                    BrakeInput = Mathf.Lerp(BrakeInput, 1, 8*Time.deltaTime);
                    if (isCarMidAir() && Input.GetKey(KeyCode.Space))
                    {
                        PIDengaged = false;
                    }  
                }
                else if (gearBox.gearEngaged == true)
                {
                    BrakeInput = Mathf.Lerp(BrakeInput, 0, 16*Time.deltaTime);
                    if (isCarMidAir() && Input.GetKey(KeyCode.Space))
                    {
                        PIDengaged = false;
                    }  
                }

                if (Input.GetAxisRaw("Vertical") == -1)
                {
                    Throttle = Mathf.Clamp(Throttle + Time.fixedDeltaTime * 1, 0, 1);
                    if (isCarMidAir() && Input.GetKey(KeyCode.Space))
                    {
                        PIDengaged = false;
                    }  
                }
                else
                {
                    Throttle = Mathf.Clamp(Throttle + Time.fixedDeltaTime * -3, 0, 1);
                }

            }
            
        }
        }

        
        
    

        
            
    }

    void GetSpeedBasedSteerAngle() // Limits steering angle in accordance with the amount of speed the car has achieved
    {
        clampedSteeringAngle = Mathf.Clamp(maxSteeringAngle *  (1.0f / (1.0f + Mathf.Abs(carSpeed) * k)),minSteeringAngle,maxSteeringAngle); // Clamp the steering angle with the minimum and maximmum, rate of k determines how quick the angle is limited
        avgBackSlipAngle = 0; // Also consider the slip angle of the rear tires, if a spinout is to occur, we should increase steering angle to accomodate
        for (int i = 2; i < wheels.Length; i++) // Take the average slip angle here
        {
            avgBackSlipAngle += wheels[i].slipAngle * Mathf.Rad2Deg / 2;
        }
        if (Mathf.Abs(avgBackSlipAngle) > clampedSteeringAngle + 5) // If the average slip angle far exceeds the max steering angle, we clamp it again
        {
            clampedSteeringAngle = Mathf.Clamp(Mathf.Abs(avgBackSlipAngle),minSteeringAngle,maxSteeringAngle);
        }
    }


    public float GetAVGWheelRPM() // Gets Average Wheel RPM of the car
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
    void GetDownForce() // Calculate downforce to be applied evenly for all 4 wheels
    {
        downForce = Mathf.Min(maxAero,0.5f*airDensity * Mathf.Pow(Mathf.Abs(carSpeed),2)*aeroRate);
        //rb.AddForceAtPosition(Vector3.down*downForce, rb.centerOfMass);
    }

    void AerialRotation() // Script for allowing mid-air control using the same inputs for on ground driving
    {


        // Allow the car to Rotate
        float h = Input.GetAxis("Horizontal") * airRotationAmount * Time.deltaTime;
        float v = Input.GetAxis("Vertical") * airRotationAmount * Time.deltaTime;

        if (Input.GetKey(KeyCode.Space))
            rb.AddTorque(rb.transform.right * v, ForceMode.Acceleration);

        if (allowBarrelRoll) // If barrel rolls are allowed, flat spins are replaced with barrel rolls instead
        {
            rb.AddRelativeTorque(0f, 0f, h, ForceMode.Acceleration);
        }
        else
        {
            rb.AddTorque(rb.transform.up * h, ForceMode.Acceleration);
        }
        //Debug.DrawRay(rb.position, new Vector3(rb.velocity.x,Physics.gravity.y, rb.velocity.z), Color.cyan);
    }

    void TrackStunts() // Tracks the various stunts a player may perform in the air.
    {
        angleDiffs = Vector3.zero; // The difference between the current and last rotation in Euler Angles
        Quaternion difference = Quaternion.Inverse(rb.rotation) * lastRotation; // Take the difference between current and last rotation (Result is in Quaternions)
        lastRotation = rb.rotation; // Update last rotation

        angleDiffs.x = Mathf.DeltaAngle(0, difference.eulerAngles.x); // Calculates the shortest distance between 0 degrees and the euler angle of the difference in all 3 dimensions 
        angleDiffs.y = Mathf.DeltaAngle(0, difference.eulerAngles.y); // (This is used to prevent getting -270 degrees instead of 90 degrees for example)
        angleDiffs.z = Mathf.DeltaAngle(0, difference.eulerAngles.z);

        totalRotation.x += angleDiffs.x; // Add the difference to the total rotation


        // Increase stunt count for when a full revolution is complete.

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

    void PID() // Function for being able to stabilize the car when in mid air, and front the car from rolling after a jump. Also can flip an upside down car if near stationary
    {
        
        if (Mathf.Abs(rb.transform.localRotation.eulerAngles.z) > 60f && Mathf.Abs(carSpeed) <= 5f) { // If the car is upside down and nearly going at 0 km/h, then we allow the player to flip the car.
        rb.AddRelativeTorque(0f, 0f, 30, ForceMode.Acceleration);
        }

        bool isHit = Physics.Raycast(rb.position, new Vector3(rb.velocity.x,Physics.gravity.y, rb.velocity.z), out RaycastDir); // Raycast, predicts where the car will land approximately.

        // Debug.DrawRay(rb.position, new Vector3(rb.velocity.x,Physics.gravity.y, rb.velocity.z), Color.cyan);


        if (isHit) // If there is a ground underneath the car...
        {

            //Debug.DrawRay(RaycastDir.point, RaycastDir.normal, Color.magenta);
            //Debug.DrawRay(rb.position, rb.transform.up, Color.magenta);


            

            Vector3 targetValue = RaycastDir.normal; // Get the normal vector of the ground, this will be our target value we want the car's rotation to match.
            //Vector3 error = targetValue - rb.transform.up;
            Vector3 error = Vector3.Cross(rb.transform.up, targetValue); // The discrepency between the target and current rotation of the car.
            // Calculate P
            Vector3 P = proportionalGain * error; // The P value that will help lessen the error (Rotate to align the car to normal)
            // Calculate D term
            Vector3 errorRateOfChange = (error - errorLast) / Time.fixedDeltaTime; // Rate of which the error changes, used for D value
            errorLast = error;


            Vector3 D = derivativeGain * errorRateOfChange; // This value is responsible for preventing an overshoot of the target.
            

                        // Calculate I term
            integrationStored = integrationStored + (error * Time.fixedDeltaTime); // This is responsible for trying to make the rotation of the car align with the target as much as possible
            Vector3 I =  integrationStored * integralGain;


            rb.AddTorque(P+D+I, ForceMode.Impulse); // Combine and add all forces as torques
        }


        // float error = targetValue - currentValue;

    }

    #endregion

    #region Nitro

    public void activateNitro()
    {
        if (nitroOn)
        {     
            applyNitro();
            nitroDelayInit = true;            
            nitroValue = Mathf.Clamp(nitroValue + Time.deltaTime * nitroHeatRate, 0, 1);
            if (nitroValue == 1 && !isOverBoosting)
            {
                Debug.Log("OverBoost Alert!");
                isOverBoosting = true;
                StartCoroutine(overBoostCountDown());
            }
        }
        
    }

    public void applyNitro()
    {
        rb.AddForce(transform.forward * kineticBoost * (kineticBoostMultiplier * (isCarMidAir() ? 1 : 0)));
        engine.nitroTorque = engine.initialTorque * nitrousPower;
    }

    private IEnumerator overBoostCountDown()
    {
        while (nitroOn)
        {
            nitroOverBoostValue += 1;
            if (nitroOverBoostValue >= maxOverBoostPenalty)
            {
                Debug.Log("Kaboom!");
            } 
            yield return new WaitForSeconds(1);

        }
        
    }

     private IEnumerator NitroReactivation()
    {
        while (nitroOn)
        {
            yield return new WaitForSeconds(1);
            
            Debug.Log("Still Boosting...");
            
        }
        nitroOverBoostValue = 0;
        nitroDelay = Mathf.Clamp(nitroDelay - 1 * Time.deltaTime , 0, 1);
        
    }
    #endregion

    
}
