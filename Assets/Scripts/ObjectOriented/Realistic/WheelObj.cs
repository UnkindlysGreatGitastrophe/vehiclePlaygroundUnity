using System;
using System.Collections;
using System.Collections.Generic;
using JetBrains.Annotations;
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
    [SerializeField] private Vector3 forcePerTire; 
    
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
    bool isHit;

    [Header("Wheel Outputs")]
    public float ReactionTorqueToWheel = 0; // N*m ->  1 kilogram meter per second squared * meters -> (kg*m/s^2)*m
    public float wheelAngularAcceleration = 0; //RADS/SEC^2
    public float wheelAngularVelocity = 0; // RADS/SEC
    public float Speed = 0; // M/S

    [Header("Wheel Vectors")]

    public Vector3 localVelocity; // M/S

    [Header("Longitudinal Variables")]
    public float longitudinalForce; // Newtons -> kg*m/s^2
    public float slipRatio;
    Vector3 dragForce; // Newtons -> kg*m/s^2
    Vector3 rollResistance; // Newtons -> kg*m/s^2
    


    [Header("Lateral Variables")]
    public float steeringAngle; 

    public float lateralForce;
    public float slipAngle;
    public float maxSteeringAngle; 
    public float tireGripFactor; 
    
    [Header("Pacejka Variables")]
    float peak = 1;
    float x_shape = 1.35f;
    public float z_shape = 1.3f;
    float stiff = 10;
    float curve = 0;

    // Start is called before the first frame update
    void Start()
    {
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
            carRigidBody.AddForceAtPosition(forcePerTire, transform.position); // ApplyForce
            updateWheelRotation();
            wheels.transform.localPosition = new Vector3(0,-(hitDistance-tireRadius),0);
        }
        else
        {
            wheels.transform.localPosition = new Vector3(0,-(maxHitDistance-tireRadius),0);
        }
    }
    void Raycast()
    {
        Debug.DrawRay(transform.position, -transform.TransformDirection(Vector3.up) * maxHitDistance, Color.yellow);         // Debug Raycast Here
        isHit = Physics.Raycast(transform.position, -transform.TransformDirection(Vector3.up), out RaycastDir, maxHitDistance); // Raycast
    }
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
        forcePerTire =  Vector3.Normalize(RaycastDir.normal) * Fz; //Why 100?

        // Generally, given a mass M, if a car has 4 springs, then each spring must be able to the produce a force of M/4 * 9.81 to be able to hold a suspension well.
        // Dampening usually is 1/10th the Stiffness Rate.
    }

    void updateWheelRotation()
    {
        wheels.transform.Rotate(wheelAngularVelocity * Time.fixedDeltaTime * Mathf.Rad2Deg,0,0); // RADS/SEC * SEC * DEG/RADS
        steeringAngle = car.steeringInput * maxSteeringAngle;
        transform.localRotation = Quaternion.Euler(0, steeringAngle*steeringFactor, 0);
    }

    #region Slip Ratio
    
    public void applyTorqueToWheels(float torqueToApply)
    {
        wheelAngularAcceleration = (torqueToApply + ReactionTorqueToWheel) / wheelInertia;
        wheelAngularVelocity += wheelAngularAcceleration * Time.fixedDeltaTime;
    }

        public void applyLongitudinalForce()
    {
        localVelocity = transform.InverseTransformDirection(carRigidBody.GetPointVelocity(RaycastDir.point));
        slipRatio = GetSlipRatio(wheelAngularVelocity, localVelocity.z);
        float driveForce = PacejkaApprox(slipRatio, z_shape);
        float dragCoefficient = 0.26f; // might be a bit too much
        dragForce = -dragCoefficient * localVelocity * localVelocity.magnitude;
        float resistanceCoefficient = 10f;
        rollResistance = -resistanceCoefficient * localVelocity;
        longitudinalForce = driveForce
        //+ dragForce 
        //+ rollResistance
        ;
        Debug.DrawRay(transform.position, (longitudinalForce * transform.forward).normalized ,Color.green);
        Debug.DrawRay(transform.position, dragForce.normalized ,Color.red);
        Debug.DrawRay(transform.position, dragForce.normalized ,Color.yellow);
        ReactionTorqueToWheel = -longitudinalForce * tireRadius; // 3rd law, needed for clutch!
        if(isHit)
            carRigidBody.AddForceAtPosition((longitudinalForce * transform.forward) + dragForce + rollResistance, transform.position);
    }



    public float PacejkaApprox(float slip,float t_shape) // This is from the godot tutorial
    {
        return Fz * peak * Mathf.Sin(t_shape * Mathf.Atan(stiff * slip - curve * (stiff * slip - Mathf.Atan(stiff * slip))));
    }

    float GetSlipRatio(float wheelVelocity, float longitudinalVelocity) 
   {
    //     float wheelLinearVelocity = wheelVelocity * tireRadius;
    //    const float fullSlipVelocity = 5.0f;

    //    if (longitudinalVelocity == 0)
    //        return 0;

    //    float absRoadVelocity = Mathf.Abs(longitudinalVelocity);
    //    float dampRatio = Mathf.Clamp01(absRoadVelocity / fullSlipVelocity);

    // 	//return (wheelLinearVelocity - longitudinalVelocity) / Mathf.Max(absRoadVelocity, 5.0f);
    //    return (wheelLinearVelocity - longitudinalVelocity) / absRoadVelocity * dampRatio;

	float relaxLength = 0.1f;

	float slipVelocity = wheelVelocity * tireRadius - longitudinalVelocity;
	float dampedAbsRoadVel = Mathf.Max(Mathf.Abs(longitudinalVelocity), 5f);
	float steadyStateSlipRatio = slipVelocity / dampedAbsRoadVel;
	float clampRatio = Mathf.Abs(steadyStateSlipRatio);
	float clampDelta = Time.fixedDeltaTime * Mathf.Abs(diffSlipRatio - steadyStateSlipRatio) / relaxLength;

	float delta = Time.fixedDeltaTime * (slipVelocity - Mathf.Abs(longitudinalVelocity) * diffSlipRatio) / relaxLength;
	delta = Mathf.Clamp(delta, -clampDelta, clampDelta);

	diffSlipRatio = Mathf.Clamp(diffSlipRatio + delta, -clampRatio, clampRatio);
	return diffSlipRatio;
   }

   #endregion

   #region Slip Angle

    public void applyLateralForce()
    {
        localVelocity = transform.InverseTransformDirection(carRigidBody.GetPointVelocity(RaycastDir.point));
        slipAngle = GetSlipAngle(wheelAngularVelocity, localVelocity.x);
        lateralForce = PacejkaApprox(slipAngle, x_shape) * tireGripFactor;
        if(isHit)
            carRigidBody.AddForceAtPosition(lateralForce * transform.right, transform.position);
    }

    float GetSlipAngle(float wheelVelocity, float lateralVelocity)
    {
        float wheelForwardVelocity = wheelVelocity * tireRadius;
        return -Mathf.Atan2(lateralVelocity, MathF.Abs(wheelForwardVelocity));
    }

   #endregion


}
