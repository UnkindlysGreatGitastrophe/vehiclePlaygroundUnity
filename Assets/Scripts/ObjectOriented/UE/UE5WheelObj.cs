using System.Collections;
using System.Collections.Generic;
using UnityEditor.Networking.PlayerConnection;
using UnityEngine;

public class UE5WheelObj : MonoBehaviour
{
    public float wheelAngularVelocity;
    public float wheelInertia;
    public UE5DifferentialObj connectedDifferential;
    private Rigidbody carRigidBody;
    public UE5CarObj UE5Car;
    private bool isHit;
    float contactDepth; float contactSpeed; float lastContactDepth;
    private float springForce;
    float maxHitDistance; float hitDistance;
    RaycastHit RaycastDir;
    private float damperForce;
    public float springRate; public float damperRate; public float suspensionRestLength; 
    private float Fz;
    [SerializeField] private Vector3 forcePerTire; 
    public GameObject wheels;
    private float tireRadius;


    // Start is called before the first frame update
    void Start()
    {
        connectedDifferential = transform.parent.GetComponent<UE5DifferentialObj>();
        carRigidBody = UE5Car.GetComponent<Rigidbody>();

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

    public void UpdatePhysic()
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
        //updateWheelRotation();
    }
}
