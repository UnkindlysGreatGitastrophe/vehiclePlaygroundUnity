using Baracuda.Monitoring;
using UnityEngine;
using UnityEngine.Animations;

public class EngineObj : MonoBehaviour
{   
    [Header("References")]
    public CarObj car;
    [Header("Engine Params")]
    [Tooltip("The torque curve the engine will follow")]
    public AnimationCurve engineCurve; // The torque curve the engine will follow
    [Tooltip("The Maximum possible RPM for the engine's natural operation")]
    public float rpmLimit = 7000; // Maximum possible RPM for the engine's natural operation
    [Tooltip("The Minimum possible RPM for the engine's natural operation")]
    public float rpmLimitIdle = 800; // Minimum possible RPM for the engine's natural operation
    [Tooltip("The resistance of the torque produced by the engine. a = F/m, but for engines it is a = Torque/Inertia")]
    public float engineMoment = 0.2f;
    [Tooltip("The drag of the engine that increases with speed, helps decelerate the engine after the throttle is released")]
    public float engineDrag = 0.02f;
    [Tooltip("The initial drag of the engine, is a constant")]
    public float engineBrake = 10f;
    [Tooltip("How far back the engine gets sent back once the redline is hit")]
    public float overRevPenalty = 500;
    public float nitroTorque = 0;

    [Header("Engine Outputs")]
    [Tooltip("The engine's angular velocity at a given time, unit is in RAD/S")]
    public float engineAngularVelocity = 0.0f;
    [Tooltip("The engine's Revolutions per Minute, measured in RPMs")]
    //[Monitor] 
    private float engineRPM = 0.0f;
    [Header("Engine Torques")]
    [Tooltip("The torque produced by the engine without any drag penalties")]
    //[Monitor] 
    public float initialTorque;
    [Tooltip("The resistance torque produced by engine drag and the engine brake")]
    //[Monitor] 
    public float dragTorque; // Resistance torque opposing the initial torque the car engine generates
    [Tooltip("The output torque, adds together both the initial torque and the resistance torques")]
    //[Monitor] 
    public float torque_out; // Output torque of the engine itself
    [Tooltip("Clutch torque, used to try and balance the engine RPM with the transmission")]
    //[Monitor] 
    public float clutch_torque;
    //[Monitor] 
    private float horsepower;


    
    [Header("Constants")]
    [Tooltip("Converts RAD/S to RPM (Approximately 9.55)")]
    public float AV_2_RPM = 60.0f / (2.0f*Mathf.PI);
    [Tooltip("Converts RPM to RAD/S (Approximately 0.105)")]
    public float RPM_2_AV;

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
        
        RPM_2_AV = 1.0f/AV_2_RPM;
    }



    #region Engine
        public void engineOperation()
        {
            if (car.status == CarObj.CarStatus.RUNNING)
            {
                initialTorque = engineCurve.Evaluate(engineRPM) * car.Throttle; // The torque the engine makes, based on RPM and throttle
                dragTorque = engineBrake + (Mathf.Abs(engineAngularVelocity)*AV_2_RPM*engineDrag); // The friction torque that counters initial torque, gets higher with RPMs
                if (Mathf.Round(engineAngularVelocity * AV_2_RPM) >= rpmLimit) // If the Engine RPM is > RPMLimit, we decrease the angular velocity, and apply no initial torque
                {
                    engineAngularVelocity -= overRevPenalty * RPM_2_AV;
                    if (!car.nitroSystem.nitroOn)
                        initialTorque = 0;
                }
                    if(car.gearBox.get_ratio() != 0.0f && car.gearBox.gearEngaged) // If not in Neutral or shifting
                    clutch_torque = car.clutch.calculateClutch(); // Function for calculating clutch Torque (TC)
                else
                {
                    clutch_torque = 0; // Clutch is not connected
                    car.clutch.clutchLock = 0;
                }
            }
            else
            {
                initialTorque = 0;
                clutch_torque = 0;
            }
            
            torque_out = initialTorque // Total Torque Output is the initial torque of engine - the drag torque - the clutch torque trying to balance the rpms with the drivetrain.
            - dragTorque * Mathf.Sign(engineAngularVelocity)
            - clutch_torque
            + nitroTorque
            ;
            float engineAccel = torque_out / engineMoment; // Update Engine Angular Velocity based on the calculated torque.
            engineAngularVelocity += engineAccel * Time.fixedDeltaTime; // Is this integration
            engineAngularVelocity = Mathf.Clamp(engineAngularVelocity, rpmLimitIdle*RPM_2_AV, rpmLimit * RPM_2_AV); // Clamp the engine angular velocity with the max and minimum RPM.
            engineRPM = engineAngularVelocity * AV_2_RPM; // Engine RPM gets updated accordingly
            horsepower = Mathf.Abs(((initialTorque // Total Torque Output is the initial torque of engine - the drag torque - the clutch torque trying to balance the rpms with the drivetrain.
            - dragTorque + nitroTorque* Mathf.Sign(engineAngularVelocity)) * engineRPM)/(7127));
        }
    #endregion
}
