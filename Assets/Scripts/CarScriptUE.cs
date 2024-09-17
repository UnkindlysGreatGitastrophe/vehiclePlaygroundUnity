using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CarScriptUE : MonoBehaviour
{
    public Wheel[] wheels;
    public Transform[] wheelTransform;

    public float[] Fz;
    public Vector3[] FzForce;

    public int numberOfAxels = 0;
    public float restLength = 0;
    public float springStiffness = 0;
    public float damperStiffness = 0;
    public float wheelRadius = 0;
    bool[] rayDidHit; 
    Rigidbody carRigidBody;
    RaycastHit[] tireRay;

    void Start()
    {
        carRigidBody = transform.GetComponent<Rigidbody>();
        wheels = new Wheel[numberOfAxels*2];
        rayDidHit = new bool[numberOfAxels*2];
        tireRay = new RaycastHit[numberOfAxels*2];
        Fz = new float[numberOfAxels*2];
        FzForce = new Vector3[numberOfAxels*2];
        for (int i = 0; i < numberOfAxels*2; i++)
        {
            Wheel wheel = new Wheel(restLength, springStiffness, damperStiffness, wheelRadius);
            wheels[i] = wheel;
        }
    }

    void Update() 
    {
        for (int i = 0; i < wheels.Length; i++){
        Vector3 raycastStart = wheelTransform[i].position;
        Vector3 raycastEnd = wheelTransform[i].transform.up * (restLength + wheelRadius);
        rayDidHit[i] = Physics.Raycast(raycastStart, -wheelTransform[i].transform.TransformDirection(Vector3.up), out tireRay[i], raycastEnd.magnitude);
        Debug.DrawRay(wheelTransform[i].position, -wheelTransform[i].TransformDirection(Vector3.up) * raycastEnd.magnitude, Color.yellow);        

        if (rayDidHit[i])
        {
            Vector3 wheelLocation = wheelTransform[i].up*wheelRadius + tireRay[i].point; 
            wheels[i].currentLength = (wheelTransform[i].position - wheelLocation).magnitude;
        }   
        else
        {
            wheels[i].currentLength = wheels[i].restLength; 
        } 

        //rayDidHit = Physics.Raycast(raycastStart, -wheelTransform[i].transform.TransformDirection(Vector3.up), out tireRay, raycastEnd.y);
        //Debug.DrawRay(wheelTransform[i].position, -wheelTransform[i].TransformDirection(Vector3.up) * raycastEnd.y, Color.yellow);            
        }
    }

    void FixedUpdate()
    {
        for (int i = 0; i< wheels.Length; i++)
        {
            if (rayDidHit[i])
            {
                float springForce = springStiffness * (wheels[i].restLength - wheels[i].currentLength);
                float damperForce = damperStiffness * (wheels[i].lastLength - wheels[i].currentLength);
                Fz[i] = springForce + damperForce;
                FzForce[i] = Fz[i] * Vector3.Normalize(tireRay[i].normal);
                wheels[i].lastLength = wheels[i].currentLength;
                carRigidBody.AddForceAtPosition(FzForce[i], wheelTransform[i].position);

            }
            else
            {
                wheels[i].currentLength = wheels[i].restLength;
                wheels[i].lastLength = wheels[i].lastLength;
                Fz[i] = 0;
                FzForce[i] = Vector3.zero;
            }
        }
    }
    public struct Wheel {
        public float restLength;
        public float currentLength;
        public float springStiffness;
        public float lastLength;
        public float damperStiffness;
        public float wheelRadius;
        float R;

        public Wheel(float restLength, float springStiffness, float damperStiffness, float wheelRadius)
        {
            this.restLength = restLength;
            this.springStiffness = springStiffness;
            this.damperStiffness = damperStiffness;
            this.wheelRadius = wheelRadius;
            this.R = wheelRadius/100;
            this.lastLength = this.restLength;
            this.currentLength = 0;
        }
    }
}
