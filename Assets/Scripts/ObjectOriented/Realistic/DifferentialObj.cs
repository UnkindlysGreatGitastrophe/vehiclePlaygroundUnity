using System;
using UnityEngine.Assertions; 
using Baracuda.Monitoring;
using UnityEngine;
using UnityEditor.EditorTools;

public class DifferentialObj : MonoBehaviour
{
    public enum DifferntialTypes {OPEN, LOCKED, SALISBURYLSD, VISCOUSLSD, TORSENLSD};

    [Header("References")]
    public CarObj car;
    public WheelObj[] connectedWheels;

    [Header("General Differential Variables")]
    [Tooltip("The type of differential that will be used to distribute torque to the car. Open diffs distribute torque evenly, which leads to one wheel overpowering the other, LSDs allows torque to be distributed to the slower wheel, while Locked diffs keep wheels speeds the same at all costs")]
    public DifferntialTypes difType = DifferntialTypes.SALISBURYLSD; // The type of differential that will distributed to the car
    [Tooltip("This torque gets applied to the differential before any sort of torque transfer applies, too little and the differential doesn't lock under any circumstances, too much, and the differential locks suddenly under any power")]
    public float preLoadTorque = 50f; // This torque gets applied to the differential before any sort of torque transfer applies, too little and the differential doesn't lock under any circumstances, too much, and the differential locks suddenly under any power

    [Header("Salisbury Limited-Slip Differential Variables")]
    [Tooltip("Ranged from 0 (Max locking) to 90 degrees (No locking), this determines the amount of torque locking provided under power.")]
    public float powerAngle; // Ranged from 0 (Max locking) to 90 degrees (No locking), this determines the amount of torque locking provided under power. 
    [Tooltip("Ranged from 0 (Max locking) to 90 degrees (No locking), this determines the amount of torque locking provided under negative power (Engine braking under deceleration, hence the coast angle).")]
    public float coastAngle; // Ranged from 0 (Max locking) to 90 degrees (No locking), this determines the amount of torque locking provided under negative power (Engine braking under deceleration, hence the coast angle). 
    [Tooltip("Used to lock differential, the more there is, the more of a locking effect will be present.")]
    public int clutchPacks; // Used to lock differential, the more there is, the more of a locking effect will be present.

    [Header("Viscous Differential Variables")]
    [Tooltip("From a range of 0 (No locking like an open Diff) to 1 (Locked), this variable limits the difference in wheel speeds when under power")]
    public float powerStiffness = 0.2f; // From a range of 0 (No locking like an open Diff) to 1 (Locked), limit the difference in wheel speeds when under power
    [Tooltip("From a range of 0 (No locking like an open Diff) to 1 (Locked), this variable limits the difference in wheel speeds when not accelerating")]
    public float coastStiffness = .2f; // From a range of 0 (No locking like an open Diff) to 1 (Locked), limit the difference in wheel speeds when not accelerating
    public float currentDiffStiffness;

    [Header("Torsen Differential Variables")]
    [Tooltip("This variable acts as a threshold to the locking mechanism of the differential, If a torque ratio of 4:1, the car will open the diff when the torque with the lesser amount is 1/4 the amount of the other wheel")]
    public float powerRatio = 4f; // powerRatio > 0, the ratio that acts as a threshold to the locking mechanism of the differential, Ex: If a torque ratio of 4:1 is present, the car will begin to open the diff when the wheel with the lesser torque is 1/4 the amoung of the torque on the left.
    [Tooltip("This variable acts as a threshold to the locking mechanism of the differential when not under power, If a torque ratio of 4:1, the car will open the diff when the torque with the lesser amount is 1/4 the amount of the other wheel")]
    public float coastRatio = 4f; // powerRatio > 0, the ratio that acts as a threshold to the locking mechanism of the differential, Ex: If a torque ratio of 4:1 is present, the car will begin to open the diff when the wheel with the lesser torque is 1/4 the amoung of the torque on the left.
    
    [Header("Differential Output")]

    [Tooltip("Boolean that indicates whether the car is under power or not")]
    public bool isPowered; // Is the car powered or nah?
    [Tooltip("Indicates whether coast or power angle is currently used")]
    public float currentDiffRatio; // This will be either coast or the power angle

    [Tooltip("Displays the current ratio of the greater torque wheel divided by the lesser torque wheel")]
    public float torqueRatio;
    [Tooltip("The maximum torque that can be used to restrict a differential at a given moment")]
    public float maxTorqueTransfer; // The maximum torque that can be used to restrict a differential
    [Tooltip("The torque that will be used to restrict one side of the differential and boost the other")]
    public float lockTorque; // The lock torque that will be applied in the end, bounded by the maxTorqueTransfer variable

    [Tooltip("The difference in wheel speed between the 2 tires, negative implies the right wheel is spinning faster than the left, and vice versa (Units in RAD/S)")]
    public float wheelSpeedDiff;

    [Tooltip("The torque originating from the gearbox that will be applied to each wheel")]
    public float torqueToLeftWheel;
    [Tooltip("The torque originating from the gearbox that will be applied to each wheel")]
    public float torqueToRightWheel;







    // Start is called before the first frame update
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
        connectedWheels = new WheelObj[transform.childCount];
        Assert.AreEqual(2, connectedWheels.Length);

        for (int i = 0; i < connectedWheels.Length; i++)
        {
            connectedWheels[i] = transform.GetChild(i).GetComponent<WheelObj>();
        }

    }

    public void calculateDifferential() // Each powered axle has a differential, we calculate how we spread the torque in accordance to the differential type here. i is the index of the differential
    {
        wheelSpeedDiff = connectedWheels[0].wheelAngularVelocity - connectedWheels[1].wheelAngularVelocity;
        if (difType != DifferntialTypes.OPEN) // If the diff type is not open, we need to do more complication processing in the Get Diff Drag function
        {
            GetDiffDrag(difType);
        }
        
        else
        {
            torqueToLeftWheel = car.torqueToAxle/2;
            torqueToRightWheel = car.torqueToAxle/2;
            connectedWheels[0].applyTorqueToWheels(torqueToLeftWheel); // Apply torque "evenly", the consequence is that one wheel spins fastest, due to having the least friction compared to every other car.
            connectedWheels[1].applyTorqueToWheels(torqueToRightWheel);
        }
    }
    


     public void GetDiffDrag( DifferntialTypes differentialType)
        {

      
            

            if (differentialType == DifferntialTypes.SALISBURYLSD) // Apply a clutch-based differential if Salisbury LSD is selected
            {
                //torqueFromTransmission it`s torque from the engine * total gear ratio
                //Angles must be from 0-90
                //Clutch packs from 0-2. 
                //Coast must not be larger angle value than Power. It will create unrealistic behaviour
                //Ideal power angle is 85
                //Ideal coast angle is 45
                //Recommended value for clutch packs is 0 or 1.


                //The following 3 variables are based off the clutch derivation formula from the ClutchObj.cs
                //float a = torqueB * inertiaA; // Take torque
                //float b = torqueA * inertiaB;
                //float c = inertiaA * inertiaB * (velocityA - velocityB) / delta;

                float a = connectedWheels[1].ReactionTorqueToWheel * connectedWheels[0].wheelInertia; // T_2 * I_1
                float b = connectedWheels[0].ReactionTorqueToWheel * connectedWheels[1].wheelInertia; // T_1 * I_1
                float c = connectedWheels[0].wheelInertia * connectedWheels[1].wheelInertia * (connectedWheels[0].wheelAngularVelocity - connectedWheels[1].wheelAngularVelocity) / Time.fixedDeltaTime; // I_1 * I_2 * (W_1 - W*2 ) / t
                
                isPowered = Math.Sign(car.torqueToAxle) > 0; // Whether or not the differential is powered determines if we can use Power or Coast Angle
                currentDiffRatio = Mathf.Max(isPowered ? Mathf.Cos(powerAngle * Mathf.Deg2Rad) : Mathf.Cos(coastAngle * Mathf.Deg2Rad)); // This is where the salisbury comes in, due to the design, we are using cosine angles to get the diff ratio based on whether the car is powered
                maxTorqueTransfer = Mathf.Max(preLoadTorque, currentDiffRatio * (1+2*clutchPacks) * Mathf.Abs(car.torqueToAxle)); // This is what we use to clamp the offset torque, based on the diff ratio, the amount of clutch packs, and the torque being supplied to tHe axle

                lockTorque = (a - b + c) / (connectedWheels[0].wheelInertia + connectedWheels[1].wheelInertia); // The algebraic equation for solving the locktorque required to even out the wheels.
                lockTorque = Mathf.Clamp(lockTorque , -maxTorqueTransfer , maxTorqueTransfer); // Clamp lock torque based on the maxTorque transfer calculated earlier
                torqueToLeftWheel = (car.torqueToAxle*0.5f)-lockTorque; // Calculate the torque to each wheel and then apply them to the wheel
                torqueToRightWheel = (car.torqueToAxle*0.5f)+lockTorque;
                connectedWheels[0].applyTorqueToWheels(torqueToLeftWheel);
                connectedWheels[1].applyTorqueToWheels(torqueToRightWheel);
            
            }
            else if (differentialType == DifferntialTypes.VISCOUSLSD) // Apply a viscous-based differential
            {
                // Viscous are essentially Locked Diffs but with a factor from 0-1 that indicates how much we want the tires to be going at the same speed
                
                isPowered = Math.Sign(car.torqueToAxle) > 0; // Whether or not the differential is powered determines if we can use Power or Coast Angle
                currentDiffStiffness = Mathf.Max(isPowered ? powerStiffness : coastStiffness); // Unlike Salisbury, we use a simple constant factor that determines how much locking we want under power or under engine braking

                float halfAngularAccel = (connectedWheels[0].wheelAngularVelocity - connectedWheels[1].wheelAngularVelocity) * 0.5f / Time.fixedDeltaTime; // Calculate the difference in angular velocity between the 2 wheels, then by delta time (To get acceleration RADS/S^2)
                float lockedTorque = halfAngularAccel * connectedWheels[0].wheelInertia * currentDiffStiffness;  // F = m*a, get the torque that will slow down one axle and speed up the other
                torqueToLeftWheel = (car.torqueToAxle * 0.5f) - lockedTorque; // Add/Subtract locked torque to the torque that will be sent to each side, then apply it
                torqueToRightWheel = (car.torqueToAxle * 0.5f) + lockedTorque;
                connectedWheels[0].applyTorqueToWheels(torqueToLeftWheel);
                connectedWheels[1].applyTorqueToWheels(torqueToRightWheel);
            }
            else if (differentialType == DifferntialTypes.TORSENLSD)
            {
                // Torsen LSDs act like a Locked Differential, right up until a torque ratio is exceeded, where it will then become an open differential until the torque levels reach under that torque ratio.

                isPowered = Math.Sign(car.torqueToAxle) > 0;
                currentDiffRatio = Mathf.Max(isPowered ? powerRatio : coastRatio); // Depending on whether we are under power or coasting (engine brake), we choose the corresponding ratio
                if (Mathf.Min(torqueToLeftWheel,torqueToRightWheel) != 0)
                    torqueRatio = Mathf.Max(torqueToLeftWheel,torqueToRightWheel) / Mathf.Min(torqueToLeftWheel,torqueToRightWheel); // Calculate the torque ratio that is currently present
                if (torqueRatio > currentDiffRatio) // If the ratio threshold is exceeded, open the differential up
                {
                    torqueToLeftWheel = car.torqueToAxle * 0.5f;
                    torqueToRightWheel = car.torqueToAxle * 0.5f;
                }
                else // Otherwise, continue locking, then apply the torque
                {
                    float halfAngularAccel = (connectedWheels[0].wheelAngularVelocity - connectedWheels[1].wheelAngularVelocity) * 0.5f / Time.fixedDeltaTime; // Calculate the difference in angular velocity between the 2 wheels, then by delta time (To get acceleration RADS/S^2)
                    float lockedTorque = halfAngularAccel * connectedWheels[0].wheelInertia;  // F = m*a, get the torque that will slow down one axle and speed up the other
                    torqueToLeftWheel = (car.torqueToAxle * 0.5f) - lockedTorque; // Add/Subtract locked torque to the torque that will be sent to each side
                    torqueToRightWheel = (car.torqueToAxle * 0.5f) + lockedTorque;
                }
                
                connectedWheels[0].applyTorqueToWheels(torqueToLeftWheel);
                connectedWheels[1].applyTorqueToWheels(torqueToRightWheel);
            }
            else // Locked Differential, always keep the wheels speeds as close as possible at all times
            {
                // This simply tries to keep the wheel differences as tight as possible, needs a low substep to work right
                // Note that we HAVE to add to one side of the axle and subtract torque from the other side, hence we divide by 2
                
                float halfAngularAccel = (connectedWheels[0].wheelAngularVelocity - connectedWheels[1].wheelAngularVelocity) * 0.5f / Time.fixedDeltaTime; // Calculate the difference in angular velocity between the 2 wheels, then by delta time (To get acceleration RADS/S^2)
                float lockedTorque = halfAngularAccel * connectedWheels[0].wheelInertia;  // F = m*a, get the torque that will slow down one axle and speed up the other
                torqueToLeftWheel = car.torqueToAxle * 0.5f - lockedTorque; // Add/Subtract locked torque to the torque that will be sent to each side
                torqueToRightWheel = car.torqueToAxle * 0.5f + lockedTorque;
                connectedWheels[0].applyTorqueToWheels(torqueToLeftWheel); // Apply the torque here.
                connectedWheels[1].applyTorqueToWheels(torqueToRightWheel);

            }
            
        } 




}
