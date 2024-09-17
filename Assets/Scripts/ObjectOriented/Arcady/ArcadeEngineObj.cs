using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ArcadeEngineObj : MonoBehaviour
{   
    [Header("References")]
    public CarObj car;
    [Header("Engine Params")]
    public AnimationCurve engineCurve;
    public float rpmLimit = 6000;
    public float rpmLimitIdle = 250;

    public float engineMoment = 0.025f;
    public float engineDrag = 0.03f;
    public float engineBrake = 10f;

    [Header("Engine Outputs")]
    public float engineAngularVelocity;
    [SerializeField] private float engineRPM = 0;

    [Header("Engine Torques")]
    public float dragTorque; // Resistance torque opposing the initial torque the car engine generates
    public float torque_out; // Output torque of the engine itself


    
    [Header("Constants")]
    public float AV_2_RPM = 60 / (2*Mathf.PI);
    public float RPM_2_AV;

    // Start is called before the first frame update
    void Start()
    {
        RPM_2_AV = 1/AV_2_RPM;
    }



    #region Engine
        public void engineOperation()
        {
            float initialTorque = engineCurve.Evaluate(engineRPM) * car.Throttle;
            dragTorque = engineBrake + (engineRPM*engineDrag);
            if (engineRPM >= rpmLimit)
            {
                engineRPM -= 500;
                initialTorque = 0;
            }
            torque_out = initialTorque 
            - dragTorque 
            - car.Tc
            ;
            
            float engineAccel = torque_out / engineMoment;
            engineAngularVelocity += engineAccel * Time.fixedDeltaTime;
            engineAngularVelocity = Mathf.Clamp(engineAngularVelocity, rpmLimitIdle * RPM_2_AV, rpmLimit * RPM_2_AV);
            engineRPM = engineAngularVelocity * AV_2_RPM;
        }
    #endregion
}
