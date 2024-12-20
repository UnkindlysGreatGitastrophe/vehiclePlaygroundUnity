using System.Collections;
using System.Collections.Generic;
using Baracuda.Monitoring;
using UnityEngine;

public class EngineObj : MonoBehaviour
{   
    [Header("References")]
    public CarObj car;
    [Header("Engine Params")]
    public AnimationCurve engineCurve;
    public float rpmLimit = 7000;
    public float rpmLimitIdle = 800;

    public float engineMoment = 0.2f;
    public float engineDrag = 0.02f;
    public float engineBrake = 10f;
    public float overRevPenalty = 500;

    [Header("Engine Outputs")]
    public float engineAngularVelocity = 0.0f;
    [Monitor] private float engineRPM = 0.0f;
    [Header("Engine Torques")]
    public float initialTorque;

    public float dragTorque; // Resistance torque opposing the initial torque the car engine generates
    public float torque_out; // Output torque of the engine itself
    public float clutch_torque;


    
    [Header("Constants")]
    public float AV_2_RPM = 60.0f / (2.0f*Mathf.PI);
    public float RPM_2_AV;

    // Start is called before the first frame update
    void Start()
    {
        this.StartMonitoring();
        RPM_2_AV = 1.0f/AV_2_RPM;
    }



    #region Engine
        public void engineOperation()
        {
            initialTorque = engineCurve.Evaluate(engineRPM) * car.Throttle; // The torque the engine makes, based on RPM and throttle
            dragTorque = engineBrake + (Mathf.Abs(engineAngularVelocity)*AV_2_RPM*engineDrag); // The friction torque that counters initial torque, gets higher with RPMs
            if (engineAngularVelocity * AV_2_RPM >= rpmLimit) // If the Engine RPM is > RPMLimit, we decrease the angular velocity, and apply no initial torque
            {
                engineAngularVelocity -= overRevPenalty * RPM_2_AV;
                initialTorque = 0;
            }
            clutch_torque = car.Tc;
            torque_out = initialTorque // Total Torque Output is the initial torque of engine - the drag torque - the clutch torque trying to balance the rpms with the drivetrain.
            - dragTorque * Mathf.Sign(engineAngularVelocity)
            - clutch_torque;
            
            float engineAccel = torque_out / engineMoment; // Update Engine Angular Velocity based on the calculated torque.
            engineAngularVelocity += engineAccel * Time.fixedDeltaTime;
            engineAngularVelocity = Mathf.Clamp(engineAngularVelocity, rpmLimitIdle*RPM_2_AV, rpmLimit * RPM_2_AV);
            engineRPM = engineAngularVelocity * AV_2_RPM;
        }
    #endregion
}
