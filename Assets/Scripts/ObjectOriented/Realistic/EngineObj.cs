using System.Collections;
using System.Collections.Generic;
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

    [Header("Engine Outputs")]
    public float engineAngularVelocity = 0.0f;
    [SerializeField] private float engineRPM = 0.0f;

    [Header("Engine Torques")]
    public float dragTorque; // Resistance torque opposing the initial torque the car engine generates
    public float torque_out; // Output torque of the engine itself


    
    [Header("Constants")]
    public float AV_2_RPM = 60.0f / (2.0f*Mathf.PI);
    public float RPM_2_AV;

    // Start is called before the first frame update
    void Start()
    {
        RPM_2_AV = 1.0f/AV_2_RPM;
    }



    #region Engine
        public void engineOperation()
        {
            float initialTorque = engineCurve.Evaluate(engineRPM) * car.Throttle; // The torque the engine makes, based on RPM and throttle
            dragTorque = engineBrake + (Mathf.Abs(engineAngularVelocity)*AV_2_RPM*engineDrag); // The friction torque that counters initial torque, gets higher with RPMs
            if (engineAngularVelocity * AV_2_RPM >= rpmLimit) // If the Engine RPM is > RPMLimit, we decrease the angular velocity, and apply no initial torque
            {
                engineAngularVelocity -= 500 * RPM_2_AV;
                initialTorque = 0;
            }
            torque_out = initialTorque // Total Torque Output is the initial torque of engine - the drag torque - the clutch torque trying to balance the rpms with the drivetrain.
            - dragTorque * Mathf.Sign(engineAngularVelocity)
            - car.Tc;
            
            float engineAccel = torque_out / engineMoment; // Update Engine Angular Velocity based on the calculated torque.
            engineAngularVelocity += engineAccel * Time.fixedDeltaTime;
            engineAngularVelocity = Mathf.Clamp(engineAngularVelocity, 0.0f, rpmLimit * RPM_2_AV);
            engineRPM = engineAngularVelocity * AV_2_RPM;
        }
    #endregion
}
