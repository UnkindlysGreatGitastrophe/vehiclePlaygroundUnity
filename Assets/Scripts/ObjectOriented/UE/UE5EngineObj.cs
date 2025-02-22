using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UE5EngineObj : MonoBehaviour
{   
    public UE5CarObj UE5Car;
    
    [Tooltip("The torque curve the engine will follow")]
    public AnimationCurve engineCurve; // The torque curve the engine will follow
    [Tooltip("The Maximum possible RPM for the engine's natural operation")]
        public float frictionCoeff = 0.02f;
    [Tooltip("The initial drag of the engine, is a constant")]
    public float startFriction = 10f;
    private float engineRPM;
    private float engineInertia;
    private float engineAngularVelocity;
    private float engineIdleRPM;
    private float engineMaxRPM;

    private float RPM2AV = (Mathf.PI * 2) / 60f; 
    private float AV2RPM = 1 / ((Mathf.PI * 2) / 60f);


    [Tooltip("How far back the engine gets sent back once the redline is hit")]

    internal void UpdatePhysic()
    {
        engineAcceleration();
    }

    private void engineAcceleration()
    {
        float currInitialTorque = engineCurve.Evaluate(engineRPM) * UE5Car.throttle;
        float frictionTorque = engineRPM * frictionCoeff + startFriction;
        float LoadTorque = UE5Car.UEClutch.clutchTorque;
        float currentEffectiveTorque = currInitialTorque - frictionTorque - LoadTorque;
        float engineAcceleration = currentEffectiveTorque / engineInertia;
        engineAngularVelocity =+ Mathf.Clamp(engineAcceleration * Time.fixedDeltaTime, engineIdleRPM*RPM2AV, engineMaxRPM*RPM2AV);
        engineRPM = engineAngularVelocity * AV2RPM;
    }

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
