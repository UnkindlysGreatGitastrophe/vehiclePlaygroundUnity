using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Car : MonoBehaviour
{
    public Transform[] tireTransform;
    public int[] steeringFactor;

    public float suspensionRestDist = 2;

    float tireGripFactor = 1f;

    float tireMass = 1;

    public float springDamper = 10;
    public float springStrength = 10;

    public float carTopSpeed = 69;

    public float maxSteeringAngle = 35;
    
    public float steeringAngle = 0;

    Vector3 steeringDir = Vector3.zero;
    float availableTorque = 0;
    Vector3 accelDir = Vector3.zero;

    bool rayDidHit; 
    Rigidbody carRigidBody;

    RaycastHit tireRay;



    // Start is called before the first frame update
    void Start()
    {
        carRigidBody = transform.GetComponent<Rigidbody>();
    }

    void Update()
    {
        for (int i = 0; i < tireTransform.Length; i++){
        rayDidHit = Physics.Raycast(tireTransform[i].position, -tireTransform[i].transform.TransformDirection(Vector3.up), out tireRay, suspensionRestDist);
        Debug.DrawRay(tireTransform[i].position, -transform.TransformDirection(Vector3.up) * suspensionRestDist, Color.yellow);            
        }
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        float accelInput = Input.GetAxis("Vertical");
        float steeringInput = Input.GetAxis("Horizontal");
       
        for (int i = 0; i < tireTransform.Length; i++)
        {

            steeringAngle = SteeringInterp(steeringInput);

            tireTransform[i].transform.localRotation = Quaternion.Euler(new Vector3(0, steeringAngle*steeringFactor[i], 0));


            if (rayDidHit)
            {
                Vector3 springDir = tireTransform[i].up;

                Vector3 tireWorldVel = carRigidBody.GetPointVelocity(tireTransform[i].position);

                float offset = suspensionRestDist - tireRay.distance;

                float vel = Vector3.Dot(springDir, tireWorldVel);

                float force = (offset*springStrength) - (vel*springDamper);

                carRigidBody.AddForceAtPosition(springDir * force, tireTransform[i].position);


            
                steeringDir = tireTransform[i].right;

                tireWorldVel = carRigidBody.GetPointVelocity(tireTransform[i].position);

                float steeringVel = Vector3.Dot(steeringDir,tireWorldVel);

                float desiredVelChange = -steeringVel * tireGripFactor;

                float desiredAccel = desiredVelChange / Time.fixedDeltaTime;

                carRigidBody.AddForceAtPosition(steeringDir * tireMass * desiredAccel, tireTransform[i].position);


                


                accelDir = tireTransform[i].forward;

                if (accelInput != 0)
                {
                    float carSpeed = Vector3.Dot(transform.forward, carRigidBody.velocity);

                    float normalizedSpeed = Mathf.Clamp01(Mathf.Abs(carSpeed)/ carTopSpeed);

                    availableTorque = 500 * accelInput;

                    carRigidBody.AddForceAtPosition(accelDir * availableTorque, tireTransform[i].position);

                           

                }
                
            }

            else 
            {

            }
            Debug.DrawRay(tireTransform[i].position, accelDir * availableTorque, Color.blue);            
                    Debug.DrawRay(tireTransform[i].position, steeringAngle * steeringDir, Color.red);     


        }
    }

    float SteeringInterp(float steeringInput)
    {
        steeringAngle = Mathf.Lerp(steeringAngle, maxSteeringAngle * steeringInput, 0.04f * (1/Time.deltaTime));
        return steeringAngle;
    }
}
