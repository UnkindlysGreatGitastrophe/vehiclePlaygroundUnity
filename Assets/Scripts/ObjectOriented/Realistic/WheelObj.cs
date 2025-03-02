using System;
using System.Collections;
using System.Collections.Generic;
using JetBrains.Annotations;
using Baracuda.Monitoring;

using UnityEngine;

public class WheelObj : MonoBehaviour
{
    [Header("References")]
    public CarObj car;
    public Rigidbody carRigidBody;

    [Header("Suspension Parameters")]
    public float springRate; public float damperRate; public float suspensionRestLength; 
    float contactDepth; float contactSpeed; float lastContactDepth; float maxHitDistance; float hitDistance;

    [Header("Suspension Forces")]
    [SerializeField] private float Fz;
    [SerializeField] private float springForce; [SerializeField] private float damperForce; 

    [Header("Suspension Vectors")]
    [SerializeField] internal Vector3 forcePerTire; 
    
    [Header("Wheel Parameters")]

    public float tireRadius; 
    public GameObject wheels;  
    public float tireOrientation;
    public float tireMass; 
    public int steeringFactor;
    public float wheelInertia;
	public float diffSlipRatio = 0.0f;

    [Header("Raycast")]
    RaycastHit RaycastDir;
    public bool isHit;

    [Header("Wheel Outputs")]
    public float ReactionTorqueToWheel = 0; // N*m ->  1 kilogram meter per second squared * meters -> (kg*m/s^2)*m
    public float wheelAngularAcceleration = 0; //RADS/SEC^2
    [Monitor]
    public float wheelAngularVelocity = 0; // RADS/SEC
    public float Speed = 0; // M/S

    [Header("Wheel Vectors")]

    public Vector3 localVelocity; // M/S

    [Header("Longitudinal Variables")]
    [Monitor] public float longitudinalForce; // Newtons -> kg*m/s^2
    [Monitor] public float slipRatio;
    public float driveForce;
    Vector3 dragForce; // Newtons -> kg*m/s^2
    Vector3 rollResistance; // Newtons -> kg*m/s^2
    


    [Header("Lateral Variables")]
    public float steeringAngle; 

    public float lateralForce;
    [Monitor] public float slipAngle;
    public float tireGripFactor; 
    // These need to be global
    float differentialSlipRatio = 0.0f;
    float differentialTanSlipAngle = 0.0f;

    // Relaxation lengths
    float relLenLongitudinal = 0.08f;
    float relLenLateral = 0.16f;

    
    [Header("Pacejka Variables")]
    float peak = 1;
    float x_shape = 1.35f;
    public float z_shape = 1.3f;
    float stiff = 10;
    float curve = 0;

    [Header("Brakes Variables")]
    public bool hasEBrake;
    public float brakeTorque;
    public float brakeBias;

    // Start is called before the first frame update
    void Start()
    {
        this.StartMonitoring();
        carRigidBody = car.GetComponent<Rigidbody>();
        wheels = transform.GetChild(0).gameObject;
        maxHitDistance = suspensionRestLength + tireRadius;
        wheelInertia = 0.5f * tireMass * tireRadius * tireRadius;
        wheels.transform.localRotation = Quaternion.Euler(new Vector3(0, tireOrientation, 0));
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        Raycast();
        if(isHit)
        {
            GetSuspensionForce();
            carRigidBody.AddForceAtPosition(forcePerTire, transform.position); // Apply Suspension Force
            wheels.transform.localPosition = new Vector3(0,-(hitDistance-tireRadius),0);
        }
        else
        {
            wheels.transform.localPosition = new Vector3(0,-(maxHitDistance-tireRadius),0);
        }
        updateWheelRotation();

    }
    void Raycast()
    {
        Debug.DrawRay(transform.position, -transform.TransformDirection(Vector3.up) * maxHitDistance, Color.yellow);         // Debug Raycast Here
        isHit = Physics.Raycast(transform.position, -transform.TransformDirection(Vector3.up), out RaycastDir, maxHitDistance); // Raycast
    }

    void updateWheelRotation()
    {
        wheels.transform.Rotate(wheelAngularVelocity * Time.fixedDeltaTime * Mathf.Rad2Deg,0,0); // RADS/SEC * SEC * DEG/RADS
        steeringAngle = car.steeringInput * car.clampedSteeringAngle;
        transform.localRotation = Quaternion.Euler(0, steeringAngle*steeringFactor, 0);
    }

    #region Forces
    void GetSuspensionForce()
    {
        hitDistance = (RaycastDir.point - transform.position).magnitude;
        Debug.DrawRay(transform.position, -transform.TransformDirection(Vector3.up) * (hitDistance - tireRadius), Color.red);         // Debug Raycast Here

        contactDepth = maxHitDistance - hitDistance;
        contactSpeed = (contactDepth - lastContactDepth) / Time.deltaTime; // Distance / Time
        lastContactDepth = contactDepth;

        springForce = contactDepth * springRate;
        damperForce = contactSpeed * damperRate;
        Fz = (springForce + damperForce) * 100;
        forcePerTire = Vector3.Normalize(RaycastDir.normal) * Fz; //Why 100?

        // Generally, given a mass M, if a car has 4 springs, then each spring must be able to the produce a force of M/4 * 9.81 to be able to hold a suspension well.
        // Dampening usually is 1/10th the Stiffness Rate.
    }

    public void applyWheelForces()
    {
        //We must modifiy the Lateral and longitudinal forces so that it does not exceed FZ * coefficient of friction for the tire, 
        if (slipRatio != 0 && slipAngle != 0)
        {
            longitudinalForce = longitudinalForce * (Mathf.Abs(slipRatio/100)/Mathf.Sqrt(Mathf.Pow(slipRatio/100,2) + Mathf.Pow(slipAngle*Mathf.Deg2Rad,2)));
            lateralForce = lateralForce * (Mathf.Abs(slipAngle*Mathf.Deg2Rad)/Mathf.Sqrt(Mathf.Pow(slipRatio/100,2) + Mathf.Pow(slipAngle*Mathf.Deg2Rad,2)));
        }
        else
        {
            longitudinalForce = 0;
            lateralForce = 0;
        }
        
        
        if(isHit)
        {
            carRigidBody.AddForceAtPosition(-transform.up*(car.downForce/4),transform.position);
            carRigidBody.AddForceAtPosition(lateralForce * transform.right, transform.position);
            carRigidBody.AddForceAtPosition((longitudinalForce * transform.forward) 
            //+ dragForce 
            //+ rollResistance
            , transform.position);
        }
    }

    #endregion



    #region Slip Ratio
    
    public void applyTorqueToWheels(float torqueToApply)
    {
        wheelAngularAcceleration = (torqueToApply + ReactionTorqueToWheel + brakeTorque) / wheelInertia;
        if (Mathf.Sign(wheelAngularVelocity) != Mathf.Sign(wheelAngularVelocity + (wheelAngularAcceleration * Time.fixedDeltaTime)) && (car.BrakeInput > 0.01f || car.eBrakeInput > 0.01f))
        {
            wheelAngularVelocity = 0;
            wheelAngularAcceleration = 0;
        }
        else
        {
            wheelAngularVelocity += wheelAngularAcceleration * Time.fixedDeltaTime;
        }

    }

        public void calculateLongitudinalForce()
    {
        localVelocity = transform.InverseTransformDirection(carRigidBody.GetPointVelocity(RaycastDir.point));
        Speed = localVelocity.magnitude * 3.6f;
        slipRatio = GetSlipRatio(wheelAngularVelocity, localVelocity.z);
        driveForce = PacejkaApprox(slipRatio, z_shape) 
        * tireGripFactor;
        if (car.hasTC)
        {
            applyTracControl();
        }
        float resistanceCoefficient = 0.007f;
        float rollResistanceForce = resistanceCoefficient * (forcePerTire.y);
        longitudinalForce = driveForce
        - rollResistanceForce
        //divide your limiter rpm by gear ratio, convert it to angular velocity and if you divide it by tyre radius you get what you want == Max speed for a gear
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
        //
        ;
        Debug.DrawRay(transform.position, (longitudinalForce * transform.forward).normalized ,Color.green);

        if (isHit)
            ReactionTorqueToWheel = -longitudinalForce * tireRadius; // 3rd law, needed for clutch!
        else
            ReactionTorqueToWheel = 0;
    }



    public float PacejkaApprox(float slip,float t_shape) // This is from the godot tutorial
    {
        return Fz * peak * Mathf.Sin(t_shape * Mathf.Atan(stiff * slip - curve * (stiff * slip - Mathf.Atan(stiff * slip))));
    }

    float GetSlipRatio(float wheelVelocity, float longitudinalVelocity) 
   {
    float slipVelocityForward = wheelAngularVelocity * tireRadius - longitudinalVelocity;
    float velocityForwardAbs = Mathf.Max(Mathf.Abs(longitudinalVelocity), 0.5f); // You can fine tune 0.5f for your own needs
    float steadyStateSlipRatio = slipVelocityForward / velocityForwardAbs;
    float slipRatioDeltaClamp = Mathf.Abs(steadyStateSlipRatio - differentialSlipRatio) / relLenLongitudinal / Time.fixedDeltaTime;
    float slipRatioDelta = (slipVelocityForward - Mathf.Abs(longitudinalVelocity) * differentialSlipRatio) / relLenLongitudinal;
    slipRatioDelta = Mathf.Clamp(slipRatioDelta, -slipRatioDeltaClamp, slipRatioDeltaClamp);

    differentialSlipRatio += slipRatioDelta * Time.fixedDeltaTime;
    differentialSlipRatio = Mathf.Clamp(differentialSlipRatio, -Mathf.Abs(steadyStateSlipRatio), Mathf.Abs(steadyStateSlipRatio));
    return differentialSlipRatio;
   }

   public void applyTracControl()
   {
        car.avgBackSlipRatio = 0;
        for (int i = 0; i < car.poweredWheels.Length; i++)
        {
            car.avgBackSlipRatio += (car.poweredWheels[i].slipRatio/100f) / car.poweredWheels.Length;
        }
        
        //car.Throttle = car.Throttle * Mathf.Clamp((0.1404f - Mathf.Abs(car.avgBackSlipRatio)) / 0.1404f, 0, 1);
        if (Mathf.Abs(car.avgBackSlipRatio) > 0.1404f)
        {
        float maxThrottle = Mathf.Clamp((0.1404f - Mathf.Abs(car.avgBackSlipRatio)) / 0.1404f, 0, 1);
        car.Throttle = Mathf.Clamp(car.Throttle,0,maxThrottle);
        }
        
   }

   #endregion

   #region Slip Angle

    public void calculateLateralForce()
    {
        localVelocity = transform.InverseTransformDirection(carRigidBody.GetPointVelocity(RaycastDir.point));
        slipAngle = GetSlipAngle(wheelAngularVelocity, localVelocity.x);
        slipAngle = CalcSlipAngle();
        slipAngle = slipAngle * Mathf.Rad2Deg;
        lateralForce = PacejkaApprox(slipAngle, x_shape) * tireGripFactor;
    }

    float GetSlipAngle(float wheelVelocity, float lateralVelocity)
    {
        float wheelForwardVelocity = wheelVelocity * tireRadius;
        return -Mathf.Atan2(lateralVelocity, MathF.Abs(wheelForwardVelocity));
    }

    float CalcSlipAngle()
    {
        localVelocity = transform.InverseTransformDirection(carRigidBody.GetPointVelocity(RaycastDir.point));
        float velocityForwardAbs = Mathf.Max(Mathf.Abs(localVelocity.z), 0.5f); // You can fine tune 0.5f for your own needs
        float steadyStateSlipAngle = Mathf.Atan2(localVelocity.x, velocityForwardAbs);
        float tanSlipAngleDeltaClamp = Mathf.Abs(Mathf.Tan(steadyStateSlipAngle) - differentialTanSlipAngle) / relLenLateral / Time.fixedDeltaTime;
        float tanSlipAngleDelta = (localVelocity.x - Mathf.Abs(localVelocity.z) * differentialTanSlipAngle) / relLenLateral;
        tanSlipAngleDelta = Mathf.Clamp(tanSlipAngleDelta, -tanSlipAngleDeltaClamp, tanSlipAngleDeltaClamp);

        differentialTanSlipAngle += tanSlipAngleDelta * Time.fixedDeltaTime;
        differentialTanSlipAngle = Mathf.Clamp(differentialTanSlipAngle, -Mathf.Abs(Mathf.Tan(steadyStateSlipAngle)), Mathf.Abs(Mathf.Tan(steadyStateSlipAngle)));
        return -Mathf.Atan(differentialTanSlipAngle);
    }

   #endregion


   #region Brakes

    public float calculateBrakeTorque(float brakeInput, float brakeBias, float maxBrakeTorque)
    {
        if (hasEBrake && car.eBrakeInput == 1)
        {
            return car.eBrakeInput * maxBrakeTorque;
        }
        else
        {
            if (car.hasABS) // ABS
            return applyABS(brakeInput, brakeBias, maxBrakeTorque);
            else
            return maxBrakeTorque * brakeBias * brakeInput;
        }
        
    }

    public float applyABS(float brakeInput, float brakeBias, float maxBrakeTorque)
    {
        if (Mathf.Abs(slipRatio/100) < 0.25f) // ABS
            return maxBrakeTorque * brakeBias * brakeInput;
            else
            return 0;
    }

   #endregion


}
