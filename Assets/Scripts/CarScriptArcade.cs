using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CarScriptArcade : MonoBehaviour
{

    [Header("References")]

    private Rigidbody carRB;
    [SerializeField] private Transform[] rayPoints;
    [SerializeField] private LayerMask driveable;
    [SerializeField] private Transform accelerationPoint;
    [SerializeField] private GameObject[] tires = new GameObject[4];
    [SerializeField] private GameObject[] frontTireParents = new GameObject[2];

    [Header("Suspension Settings")]
    public float springStiffness; // Max force spring can exert when fully compressed
    public float damperStiffness;
    public float restLength; // Standard length of spring when not compressed or stretched
    public float springTravel; // Maximum distance our spring can compress or extend from its normal rest position
    public float wheelRadius;

    private int[] wheelsIsGrounded = new int[4];
    private bool isGrounded = false;

    [Header("Input")]
    public float moveInput = 0;
    public float steerInput = 0;
    private float brakeInput = 0;
    [Header("Car Settings")]
    [SerializeField] private float acceleration = 25;
    [SerializeField] private float maxSpeed = 100;
    [SerializeField] private float deceleration = 10;

    [SerializeField] private float brakeForce = 31;

    private Vector3 currentCarLocalVelocity = Vector3.zero;
    private float carVelocityRatio = 0;
    [SerializeField] private float steerStrength;
    [SerializeField] private AnimationCurve turningCurve;
    [SerializeField] private float DragCoefficient = 1f;

    [Header("Visuals")]
    [SerializeField] private float tireRotSpeed = 3000f;
    [SerializeField] private float maxSteeringAngle = 30f;

    // Start is called before the first frame update
    void Start()
    {
        carRB = GetComponent<Rigidbody>();
    }

    void Update()
    {
        GetPlayerInput();    
    }

    void FixedUpdate()
    {
        Suspension();
        GroundCheck();
        CalculateCarVelocity();
        Movement();
        Visuals();
    }
    private void Suspension()
    {
        for (int i = 0; i < rayPoints.Length; i++)
        {
            wheelsIsGrounded[i] = 1;
            RaycastHit hit;
            float maxLength = restLength + springTravel;
            if (Physics.Raycast(rayPoints[i].position, -rayPoints[i].up, out hit, maxLength + wheelRadius, driveable))
            {
                float currentSpringLengtht = hit.distance - wheelRadius;
                float springCompression = (restLength - currentSpringLengtht) / springTravel;
                float springVelocity = Vector3.Dot(carRB.GetPointVelocity(rayPoints[i].position), rayPoints[i].up);
                float dampForce = damperStiffness * springVelocity;
                float springForce = springStiffness * springCompression;
                float netForce = springForce - dampForce;
                carRB.AddForceAtPosition(netForce * rayPoints[i].up, rayPoints[i].position);
                Debug.DrawLine(rayPoints[i].position, hit.point, Color.red);
                // Visuals
                SetTirePosition(tires[i], hit.point + rayPoints[i].up * wheelRadius);

            }
            else
            {
                wheelsIsGrounded[i] = 0;
                // Visuals
                SetTirePosition(tires[i], rayPoints[i].position - rayPoints[i].up*maxLength);
                Debug.DrawLine(rayPoints[i].position, rayPoints[i].position + (wheelRadius + maxLength) * -rayPoints[i].up, Color.green);
            }
        }
        
    }

    #region Car Status Check
    private void GroundCheck()
    {
        int  tempGroundedWheels = 0;
        for (int i = 0; i < wheelsIsGrounded.Length;i++)
        {
            tempGroundedWheels += wheelsIsGrounded[i];
        }
        if (tempGroundedWheels > 1)
        {
            isGrounded = true;
        }
        else
        {
            isGrounded = false;
        }
    }

    private void CalculateCarVelocity()
    {
        currentCarLocalVelocity = transform.InverseTransformDirection(carRB.velocity);
        carVelocityRatio = currentCarLocalVelocity.z / maxSpeed;
    }
    #endregion
    
    #region Input Handling

    private void GetPlayerInput()
    {
        moveInput = Input.GetAxis("Throttle");
        steerInput = Input.GetAxis("Horizontal");
        brakeInput = Input.GetAxis("Brake");
    }

    #endregion

    #region Movement

    private void Movement()
    {
        if (isGrounded)
        {
            Acceleration();
            Deceleration();
            Brakes();
            Turn();
            SidewaysDrag();
        }

    }
    private void Acceleration()
    {
        carRB.AddForceAtPosition(acceleration * moveInput * transform.forward, accelerationPoint.position, ForceMode.Acceleration);
    }

    private void Deceleration()
    {
        carRB.AddForceAtPosition(deceleration * moveInput * -transform.forward, accelerationPoint.position, ForceMode.Acceleration);
    }

    private void Turn()
    {
        carRB.AddTorque(steerStrength * steerInput * turningCurve.Evaluate(carVelocityRatio) * Mathf.Sign(carVelocityRatio) * transform.up, ForceMode.Acceleration);
    }

    private void SidewaysDrag()
    {
        float currentSidewaysSpeed = currentCarLocalVelocity.x;
        float dragMagnitude = -currentSidewaysSpeed * DragCoefficient;
        Vector3 dragForce = transform.right * dragMagnitude;
        carRB.AddForceAtPosition(dragForce, carRB.worldCenterOfMass, ForceMode.Acceleration);
    }

    private void Brakes()
    {
        carRB.AddForceAtPosition(deceleration * brakeInput * -transform.forward, accelerationPoint.position, ForceMode.Acceleration);
    }
    #endregion

    #region Visuals

    private void Visuals()
    {
        TireVisuals();
    }

    private void TireVisuals()
    {
        float steeringAngle = maxSteeringAngle * steerInput;
        for (int i = 0; i < tires.Length; i++)
        {
            if (i < 2)
            {
                tires[i].transform.Rotate(Vector3.right, tireRotSpeed * carVelocityRatio * Time.deltaTime, Space.Self);
                frontTireParents[i].transform.localEulerAngles = new Vector3(frontTireParents[i].transform.localEulerAngles.x, steeringAngle, frontTireParents[i].transform.localEulerAngles.z);
            }
            else
            {
                tires[i].transform.Rotate(Vector3.right, tireRotSpeed * moveInput * Time.deltaTime, Space.Self);
            }
        }
    }

    private void SetTirePosition(GameObject tire, Vector3 targetPosition)
    {
        tire.transform.position = targetPosition;
    }
    #endregion
}
