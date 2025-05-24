using System;
using System.Collections;
using System.Collections.Generic;
using Baracuda.Monitoring;
using UnityEngine;
using UnityEngine.Assertions;

public class GearBoxObj : MonoBehaviour
{
    public enum GearboxType {MANUAL, AUTOMATIC};
    [Header("References")]
    public CarObj car;

    [Header("GearBox Params")]
    [Tooltip("Total amount of gears in the gearbox")]
    public int numOfGears;
    [Tooltip("The exact gear ratios")]
    public float[] gearRatios;
    [Tooltip("The time to upshift a gear (In seconds)")]
    public float shiftTime = 1;
    [Tooltip("The gear ratio that is applied to all gear ratios")]
    public float finalDriveGear = 3.6f;
    [Tooltip("Automatic or Manual?")]
    public GearboxType Transmissiontype;



    [Header("GearBox Outputs")]
    [Tooltip("The current gear the car is using, 0 = Reverse, 1 = Neutral")]
    public int currentGear = 1;
    [Tooltip("Bool that determines if the gear is currently engaged, if not, this means that the clutch is not active, and the throttle may not be active either")]
    [SerializeField] public bool gearEngaged = true;
    private float oldCarSpeed;

    void Awake()
    {
            numOfGears = gearRatios.Length; // Dynamically assign numOfGears based on gearRatios array
    }
    void Start(){

        // DEBUGGING
        this.StartMonitoring();
        if (transform.parent.GetComponent<CarObj>() != null)
        {
            car = transform.parent.GetComponent<CarObj>();
        }
        if (transform.parent.parent.GetComponent<CarObj>() != null)
        {
            car = transform.parent.parent.GetComponent<CarObj>();
        }
        // VARIABLE INITIALIZING
        numOfGears = gearRatios.Length; // Dynamically assign numOfGears based on gearRatios array
        // REFERENCING
        car = transform.GetComponentInParent<CarObj>();

}
    // Update is called once per frame
    void Update()
    {
        // Allows automatic gearbox transmission management, upshifting and downshifting manually is still possible however.
        if (Transmissiontype == GearboxType.AUTOMATIC)
        {
            AutoGearBoxManager();
        }
        if (Input.GetKeyDown(KeyCode.Q))
        {
            GearUP();
        }
        if (Input.GetKeyDown(KeyCode.E))
        {
            GearDOWN();
        }
    }


    #region Automatic
    private void AutoGearBoxManager() // Function responsible for managing the automatic gearbox
    {
        // If the gear is actively engaged, then only then can we allow shifting
        if (gearEngaged && !car.isCarMidAir())
        {
            // If the car speed slow enough and is flipped, and the user is pressing the brake button, and the car is not in reverse mode, enable reversing mode.
            if (Mathf.Abs(car.carSpeed) < 1 && Mathf.Sign(oldCarSpeed) != Mathf.Sign(car.carSpeed) && Input.GetAxisRaw("Vertical") == -1 && car.AutoReverseMode == false && Mathf.Abs(car.carSpeed) < 1)
            {
                car.AutoReverseMode = true; // This will invert the controls of the brake and gas
                GearREVERSE(); // Switch the gear to reverse
                //Debug.Log("Gear To Reverse");
            }
            // If the car speed slow enough and is flipped, and the user is pressing the gas button, and the car IS in reverse mode, disable reversing mode.
            else if (Mathf.Abs(car.carSpeed) < 1 && Mathf.Sign(oldCarSpeed) != Mathf.Sign(car.carSpeed) && Input.GetAxisRaw("Vertical") == 1 && car.AutoReverseMode == true)
            {
                car.AutoReverseMode = false; // Invert the controls back to normal
                GearToFirst(); // Switch to first gear
                // Debug.Log("Gear To First");
            }
            
            else if (currentGear != 0 && car.carSpeed < 0.85 *GetMaxGearSpeed(currentGear - 1)  && currentGear > 2) // If the car is NOT in reverse gear, is gear 2 or higher, and the car speed is 85% or less than the maximum speed of the previous gear, we downshift.
            {
                GearDOWN();
                //Debug.Log("Gear Down");
            }
            else if (!car.nitroSystem.nitroOn)
            {
                if (Mathf.Abs(car.calculateAVGPoweredSlipRatio()) < (.10f + (.10f *car.poweredWheels[0].tireGripFactor/2)) && (car.engine.engineAngularVelocity * car.engine.AV_2_RPM) > (car.engine.rpmLimit*0.99f) && currentGear != 0) // If the maximum gear speed approximation is reached from the car, and the gear is NOT reverse gear, then we upshift a gear
                {
                    GearUP();
                    //Debug.Log("Gear Up");
                }
            }
            else 
            {
                if (car.carSpeed >= 0.9f * Mathf.Abs(GetMaxGearSpeed(currentGear)) && currentGear != 0) // If the maximum gear speed approximation is reached from the car, and the gear is NOT reverse gear, then we upshift a gear
                {
                    GearUP();
                    //Debug.Log("Gear Up");
                }
            }
            
            oldCarSpeed = car.carSpeed;
        }
        
    }


    #endregion

    #region GearBox
    private void GearUP() // Code for upshifting a gear
    {
        if (currentGear < numOfGears-1 && gearEngaged) // Boundary checking (0 <= gear < numOfGears), also check if the gear is actually engaged and clutch is connected
        {
            //gearEngaged = false; // Disengages the clutch from the car, disconnects the flow of torque from engine to gearbox
            //Debug.Log(GetMaxGearSpeed(currentGear + 1)*0.94f); 
            StartCoroutine(NormalShift(currentGear + 1)); // Begin the shifting process
        }
    }

    private void GearToFirst() // Code to shift gear to first (Useful for automatic transmission)
    {
        if (currentGear < numOfGears-1 && gearEngaged) // Boundary checking (0 <= gear < numOfGears)
        {
            //gearEngaged = false;
            //Debug.Log(GetMaxGearSpeed(currentGear + 1));
            StartCoroutine(NormalShift(2)); // Begin the shifting process
        }
    }
    
    private void GearREVERSE() // Code to shift gear to reverse (Useful for automatic transmission)
    {
        if (gearEngaged) // Check to see if the gears correctly engaged 
        {
        //gearEngaged = false; // Disengage clutch, gear is no longer active
        //Debug.Log(GetMaxGearSpeed(currentGear - 1));
        StartCoroutine(NormalShift(0)); // Start the shift to reverse
        }
    }

    private void GearDOWN()
    {
        if (currentGear > 0 && gearEngaged) // Boundary check, check if gear is engaged
        {
        //Debug.Log(GetMaxGearSpeed(currentGear - 1));
        StartCoroutine(ShiftRevMatch(currentGear - 1)); // Attempt to rev match the shift (Quicker than normal process, but may not always complete)
        }
    }

    private float GetMaxGearSpeed(int gear)
    {
        /*
            EXAMPLE:
                    let's say you have a tyre of a radius of 0.45
                    the circumference is more or less 2.8 meters - that means the tyre is traveling 2.8 meters per revolution
                    max engine revs = 6800
                    revolutions per minute
                    divide that by 60 and you get around 113 revs per second
                    then taking into account gear ratios the wheel rotates more or less 7.8 times a second
                    2.8 meters per revolution, 7.8 revolutions per second - 22.3 meters per second
                    multiply by 3.6 - 80 KMH
                    that's the correct calculation
        */
        if (car.gearBox.get_specific_ratio(gear) == 0) // If gear is set to neutral, don't return anything (No max top speed when you cant accelerate)
        {
            return 0;
        }
        float maxRPS = (car.engine.rpmLimit - car.engine.overRevPenalty) / 60; // Maximum Revs per second
        float wheelRPS = maxRPS / car.gearBox.get_specific_ratio(gear) / car.gearBox.finalDriveGear; // Wheel Revs Per Second (Rad/s)
        float tireCircumference = car.wheels[0].tireRadius * 2 * Mathf.PI; // Assuming all tires are the same radius (Not good for dragsters)
        float metersPS = tireCircumference * wheelRPS; // Convert to meters per second (m * Rad/s)
        float GetMaxGearSpeed = metersPS * 3.6f; // UNIT IS KM/H
        float dragForce = 0.353f * GetMaxGearSpeed * GetMaxGearSpeed; // Take into account the resistance forces that are present at this speed
        float rollResistance = 0;
        for (int i = 0; i < car.differential.Length; i++)
        {
            for (int j = 0; j < car.differential[i].connectedWheels.Length; j++)
            {
                rollResistance += car.differential[i].connectedWheels[j].forcePerTire.y * 0.007f;
            }
        }
        float resistanceDecel = 3.6f * ((dragForce + rollResistance)/car.GetComponent<Rigidbody>().mass); // Turn the force into deceleartion 
        GetMaxGearSpeed = GetMaxGearSpeed - (Time.fixedDeltaTime * resistanceDecel); // Calculate maximum gear speed  with consideration of deceleration force

        return GetMaxGearSpeed;
    }

    private IEnumerator NormalShift(int shiftDirection) // The normal Upshifting procedure
    {
        currentGear = 1; // Shift to neutral
        gearEngaged = false;
        car.Throttle = 0;
        yield return new WaitForSeconds(shiftTime);
        currentGear = shiftDirection;
        gearEngaged = true;

    }

        private IEnumerator ShiftRevMatch(int shiftDirection) // The downshifting procedure, unlike normal shifts it might not always 
    {
        int previousGear = currentGear;
        gearEngaged = false; // Disconnect clutch, disengage gear
        currentGear = 1;
        car.Throttle = 0; // First disengage gear
        
        if (previousGear != 1 && car.carSpeed < GetMaxGearSpeed(shiftDirection)) // If we were not originally in neutral, and the car speed is within a range of 0 to the max gear speed of the gear we want to shift to..
        {
            //Debug.Log("Rev Matching");
            // ....Then we can do a rev match, find the difference of the Engine and transmission RPM
            float RPMDifference = car.engine.engineAngularVelocity * car.engine.AV_2_RPM - Mathf.Abs(car.GetAVGWheelRPM() * car.gearBox.get_specific_ratio(shiftDirection) * car.gearBox.finalDriveGear);
            while ( Mathf.Abs(RPMDifference) > 800) // If the difference is not close enough, then we adjust accordingly
            {
                if (RPMDifference > 0) // If engine is quicker than wheel RPM, lower throttle
                {
                    car.Throttle = Mathf.Clamp(car.Throttle + Time.fixedDeltaTime * -5, 0, 1);
                }
                else  //If wheel RPM is quicker than engine, increase throttle
                {
                    car.Throttle = Mathf.Clamp(car.Throttle + Time.fixedDeltaTime * 3, 0, 1);
                }
                // The Engine RPM will change based on what we do to throttle, lessening the difference
                RPMDifference = car.engine.engineAngularVelocity * car.engine.AV_2_RPM - (car.GetAVGWheelRPM() * car.gearBox.get_specific_ratio(shiftDirection) * car.gearBox.finalDriveGear); 
                yield return null;
            }
            // Once we are at an acceptable range for a smooth shift, complete the shift
            currentGear = shiftDirection;  
            gearEngaged = true;
        }
        else // Do a default shift if the previous condition can not be satisfied
        {
            currentGear = 1;
            car.Throttle = 0;
            yield return new WaitForSeconds(shiftTime);
            currentGear = shiftDirection;
            gearEngaged = true;
        }

    }




    public float get_ratio() // Get Current Get ratio
    {
        return gearRatios[currentGear];
    }

        public float get_specific_ratio(int gear) // Get Ratio of a desired gear, assets that the= gear is within the array of gears.
    {
        Assert.IsTrue(gear >= 0 && gear < numOfGears);
        return gearRatios[gear];
    }
    #endregion
}
