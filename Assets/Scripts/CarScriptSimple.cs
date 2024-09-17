using System;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEditor.Callbacks;
using UnityEngine;
using UnityEngine.Experimental.GlobalIllumination;

public class CarScriptSimple : MonoBehaviour
{
    [Header("Suspensaion Settings")]
    float contactDepth; float contactSpeed; float[] lastContactDepth; float R; float maxHitDistance; float hitDistance;
    float springForce; float damperForce; public float suspensionRestLength; 
    public float tireMass; public int[] steeringFactor; public float springRate; public float damperRate; 
    [Header("Wheel Parameters")]
    float steeringAngle; public float maxSteeringAngle; public float tireGripFactor; public float tireRadius; public GameObject[] wheels;  public float[] tireOrientation;
    float wheelAngularVelocity;
    [Header("References")]
    Rigidbody carRigidBody;

    [Header("Forces")]
    public Vector3[] forcePerTire; float[] Fz;
    [Header("Raycast")]
    RaycastHit[] RaycastDir;
    bool[] isHit;
    [Header("Vectors")]
    Vector3 linearVelocityLocal;
    private Vector3 accelDir; private Vector3 tireWorldVel;
 

    [Header("Input")]
    public float Throttle;
    public float Brake;
    float steeringInput;
    public float engineIdleRPM;
    public float EngineMaxRPM;

    public float driveForce = 0;




    // Start is called before the first frame update
    void Start()
    {
        Initialization();
    }

    // Update is called once per frame
    void Update()
    {
        
            
    }

    void FixedUpdate()
    {
        GetThrottle();
        
        steeringInput = Input.GetAxis("Horizontal");
        for (int i = 0; i < wheels.Length; i++)
        {
            steeringAngle = SteeringInterp(steeringInput);
            Raycast(i);
            if(isHit[i])
            {
                GetSuspensionForce(i);
                carRigidBody.AddForceAtPosition(forcePerTire[i], wheels[i].transform.position); // ApplyForce
                AddLateralForce(i);
                AddLongitudinalForce(i);
                UpdateWheelMesh(i);
                wheels[i].transform.GetChild(0).localPosition = new Vector3(0,-(hitDistance-tireRadius),0);
            }
            else
            {
                wheels[i].transform.GetChild(0).localPosition = new Vector3(0,-(maxHitDistance-tireRadius),0);
            }
            wheels[i].transform.localRotation = Quaternion.Euler(new Vector3(0, steeringAngle*steeringFactor[i], 0));
            Transform rims = wheels[i].transform.GetChild(0).transform;
            float tireRotation = wheelAngularVelocity * Mathf.Rad2Deg * Time.deltaTime;
            rims.Rotate(new Vector3(tireRotation, 0, 0));
        }
    }

    void Initialization()
    {
        carRigidBody = transform.GetComponent<Rigidbody>();
        forcePerTire = new Vector3[wheels.Length];
        RaycastDir = new RaycastHit[wheels.Length];
        isHit = new bool[wheels.Length];
        lastContactDepth = new float[wheels.Length];
        maxHitDistance = suspensionRestLength + tireRadius;
        R = tireRadius;
        Fz = new float[wheels.Length];
        for  (int i = 0; i < wheels.Length; i++)
        {
            Transform rims = wheels[i].transform.GetChild(0).transform;
            rims.localRotation = Quaternion.Euler(new Vector3(0, tireOrientation[i], 0));
        }
    }   

    void Raycast(int i)
    {
        //Debug.DrawRay(wheels[i].transform.position, -wheels[i].transform.TransformDirection(Vector3.up) * maxHitDistance, Color.yellow);         // Debug Raycast Here
        isHit[i] = Physics.Raycast(wheels[i].transform.position, -wheels[i].transform.TransformDirection(Vector3.up), out RaycastDir[i], maxHitDistance); // Raycast
    }

    void GetSuspensionForce(int i)
    {
        hitDistance = (RaycastDir[i].point - wheels[i].transform.position).magnitude;
        Debug.DrawRay(wheels[i].transform.position, -wheels[i].transform.TransformDirection(Vector3.up) * (hitDistance - tireRadius), Color.red);         // Debug Raycast Here

        contactDepth = maxHitDistance - hitDistance;
        contactSpeed = (contactDepth - lastContactDepth[i]) / Time.deltaTime; // Distance / Time
        lastContactDepth[i] = contactDepth;

        springForce = contactDepth * springRate;
        damperForce = contactSpeed * damperRate;
        Fz[i] = springForce + damperForce;
        forcePerTire[i] =  Vector3.Normalize(RaycastDir[i].normal) * (Fz[i]*100); 

        // Generally, given a mass M, if a car has 4 springs, then each spring must be able to the produce a force of M/4 * 9.81 to be able to hold a suspension well.
        // Dampening usually is 1/10th the Stiffness Rate.
    }

    float SteeringInterp(float steeringInput)
    {
        steeringAngle = Mathf.Lerp(steeringAngle, maxSteeringAngle * steeringInput, 0.04f * (1/Time.deltaTime));
        return steeringAngle;
    }

    void AddLateralForce(int i)
    {
        Vector3 steeringDir = wheels[i].transform.right;
        Debug.DrawRay(wheels[i].transform.position,steeringDir, Color.red);
        tireWorldVel = carRigidBody.GetPointVelocity(wheels[i].transform.position);
        Debug.DrawRay(wheels[i].transform.position,tireWorldVel.normalized, Color.green);
        float steeringVel = Vector3.Dot(steeringDir, tireWorldVel);
        float desiredVelChange = -steeringVel * tireGripFactor;
        float desiredAccel = desiredVelChange / Time.fixedDeltaTime;
        Debug.DrawRay(wheels[i].transform.position, (steeringDir * tireMass * desiredAccel)-tireWorldVel, Color.cyan);
        carRigidBody.AddForceAtPosition(steeringDir * tireMass * desiredAccel, wheels[i].transform.position);
    }

    void AddLongitudinalForce(int i) // In accordance with Asawicki.info
    {
        // Ftraction = u * Engineforce,
        accelDir = transform.forward;
        Vector3 engineForce = 3000 * Throttle * accelDir;
        Vector3 brakeForce = -6000 * Brake * accelDir;
        float dragCoefficient = 10f;
        Vector3 dragForce = -dragCoefficient * carRigidBody.velocity * carRigidBody.velocity.magnitude;
        float resistanceCoefficient = 10f;
        Vector3 rollResistance = -resistanceCoefficient * carRigidBody.velocity;
        Vector3 longitudinalForce = engineForce + dragForce + rollResistance + brakeForce;
        Debug.Log(carRigidBody.velocity.magnitude*3.6f + "m/s");
        carRigidBody.AddForceAtPosition(longitudinalForce, wheels[i].transform.position);
    }

    void AddLongitudinalForce2(int i) // In accordance with Asawicki.info
    {
        //TBD
    }

    void UpdateWheelMesh(int i)
    {
        if (isHit[i])
        {
            wheels[i].transform.GetChild(0).localPosition = new Vector3(0,-(hitDistance-tireRadius),0);
        }
        else
        {
            wheels[i].transform.GetChild(0).localPosition = new Vector3(0,-(maxHitDistance-tireRadius),0);
        }
        wheels[i].transform.localRotation = Quaternion.Euler(new Vector3(0, steeringAngle*steeringFactor[i], 0));
        Transform rims = wheels[i].transform.GetChild(0).transform;
        float tireRotation = wheelAngularVelocity * Mathf.Rad2Deg * Time.deltaTime;
        rims.Rotate(new Vector3(tireRotation, 0, 0));
        tireRotation = linearVelocityLocal.z/R * Mathf.Rad2Deg * Time.deltaTime * -1;
        rims.Rotate(new Vector3(tireRotation, 0, 0));
        
    }

    public void GetThrottle()
    {
            //Throttle = Input.GetButton("w");
            if (Input.GetAxisRaw("Vertical") == 1)
            {
                //Throttle = 1;
                Throttle = Mathf.Lerp(Throttle, 1, 4*Time.deltaTime);
            }
            else
            {
                //Throttle = 0;
                Throttle = Mathf.Lerp(Throttle, 0, 10*Time.deltaTime);
            }
            if (Input.GetAxisRaw("Vertical") == -1)
            {
                //Throttle = 1;
                Brake = Mathf.Lerp(Brake, 1, 8*Time.deltaTime);
            }
            else
            {
                //Throttle = 0;
                Brake = Mathf.Lerp(Brake, 0, 16*Time.deltaTime);
            }
    }

}