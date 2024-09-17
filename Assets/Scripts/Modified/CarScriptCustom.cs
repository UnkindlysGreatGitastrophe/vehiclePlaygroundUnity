using System;
using System.Collections;
using UnityEngine;
using UnityEngine.UIElements;

public class CarScriptCustom : MonoBehaviour
{
    [Header("Suspensaion Settings")]
    float contactDepth; float contactSpeed; float[] lastContactDepth; float R; float maxHitDistance; float hitDistance;
    float springForce; float damperForce; public float suspensionRestLength; 
    public float tireMass; public int[] steeringFactor; public float springRate; public float damperRate; 
    [Header("Wheel Parameters")]
    float steeringAngle; public float maxSteeringAngle; public float tireGripFactor; public float tireRadius; public GameObject[] wheels;  public float[] tireOrientation;
    private Transform carRims;


    private float wheelInertia;
    public float[] wheelAngularVelocity;
    public float Speed;

    [Header("References")]
    Rigidbody carRigidBody;

    [Header("Forces")]
    public Vector3[] forcePerTire; float[] Fz;
    [Header("Raycast")]
    RaycastHit[] RaycastDir;
    bool[] isHit;
    [Header("Vectors")]
    private Vector3 accelDir; private Vector3 tireWorldVel;
 

    [Header("Input")]
    public float Throttle;
    public float Brake;
    float steeringInput;


    [Header("Engine")]
    public AnimationCurve engineCurve;
    public float engineRPM = 0;
    public float rpmLimit = 6000;
    public float rpmLimitIdle = 250;

    public float engineMoment = 0.025f;
    public float engineDrag = 0.03f;
    public float engineBrake = 10f;
    public float dragTorque;
    public float torque_out;
    private float AV_2_RPM;
    private float RPM_2_AV;

    public float engineAngularVelocity;

    [Header("GearBox")]

    public int numOfGears;
    public float[] gearRatios;
    public int currentGear = 1;
    private bool gearEngaged = true;
    private float shiftTime = 1;
    public float netDrive;
    public float[] driveForce;

    public float finalDriveGear = 3.6f;

    public float netForce;

    [Header("Brakes")]
    public float brakeStrength = 6500;
    public float[] brakeForce;
    public float brakeBias = 1;

    [Header("Clutch")]


    public float frictionConstant=25;
    public float Tc; // Clutch Drag Torque


    // Start is called before the first frame update
    void Start()
    {
        Initialization();
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Q))
        {
            GearUP();
        }
        if (Input.GetKeyDown(KeyCode.E))
        {
            GearDOWN();
        }
    }

    void FixedUpdate()
    {
        GetThrottle();
        steeringInput = Input.GetAxis("Horizontal");
		steeringAngle = SteeringInterp(steeringInput); // No need to do that in the for loop, there's only one value for all wheels

        for (int i = 0; i < wheels.Length; i++) // First we do all the suspension stuff and calculate load for each wheel
        {
            Raycast(i);
            if(isHit[i])
            {
                GetSuspensionForce(i);
                carRigidBody.AddForceAtPosition(forcePerTire[i], wheels[i].transform.position); // ApplyForce
                wheels[i].transform.GetChild(0).localPosition = new Vector3(0,-(hitDistance-tireRadius),0);
            }
            else
            {
                wheels[i].transform.GetChild(0).localPosition = new Vector3(0,-(maxHitDistance-tireRadius),0);
            }
            calculateBrakes(i);
            UpdateWheelMesh(i);
        }
        engineUnreal();
        if (get_ratio() != 0)
        {
            Clutch();
            engageDriveTrain(); // Apply the drive force to the wheels      
        }
            
		
    }

    
    void Raycast(int i)
    {
        //Debug.DrawRay(wheels[i].transform.position, -wheels[i].transform.TransformDirection(Vector3.up) * maxHitDistance, Color.yellow);         // Debug Raycast Here
        isHit[i] = Physics.Raycast(wheels[i].transform.position, -wheels[i].transform.TransformDirection(Vector3.up), out RaycastDir[i], maxHitDistance); // Raycast
    }

    #region Init

    void Initialization()
    {
        brakeBias = Mathf.Clamp(brakeBias,0,1);
        numOfGears = gearRatios.Length;
        AV_2_RPM = 60 / (2*Mathf.PI);
        RPM_2_AV = 1/AV_2_RPM;
        carRigidBody = transform.GetComponent<Rigidbody>();
        forcePerTire = new Vector3[wheels.Length];
        RaycastDir = new RaycastHit[wheels.Length];
        isHit = new bool[wheels.Length];
        lastContactDepth = new float[wheels.Length];
        wheelAngularVelocity = new float[wheels.Length];
        maxHitDistance = suspensionRestLength + tireRadius;
        R = tireRadius;
        Fz = new float[wheels.Length];
        wheelInertia = 0.5f * tireMass * tireRadius * tireRadius;
        driveForce = new float[wheels.Length];
        brakeForce = new float[wheels.Length];
        for  (int i = 0; i < wheels.Length; i++)
        {
            Transform rims = wheels[i].transform.GetChild(0).transform;
            rims.localRotation = Quaternion.Euler(new Vector3(0, tireOrientation[i], 0));
        }
    }   

    #endregion

    #region Forces
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
        forcePerTire[i] =  Vector3.Normalize(RaycastDir[i].normal) * (Fz[i]*100); //Why 100?

        // Generally, given a mass M, if a car has 4 springs, then each spring must be able to the produce a force of M/4 * 9.81 to be able to hold a suspension well.
        // Dampening usually is 1/10th the Stiffness Rate.
    }



    void AddLateralForce(int i)
    {
        Vector3 steeringDir = wheels[i].transform.right;
        //Debug.DrawRay(wheels[i].transform.position,steeringDir, Color.red);
        tireWorldVel = carRigidBody.GetPointVelocity(wheels[i].transform.position);
        //Debug.DrawRay(wheels[i].transform.position,tireWorldVel.normalized, Color.green);
        float steeringVel = Vector3.Dot(steeringDir, tireWorldVel);
        float desiredVelChange = -steeringVel * tireGripFactor;
        float desiredAccel = desiredVelChange / Time.fixedDeltaTime;
        //Debug.DrawRay(wheels[i].transform.position, (steeringDir * tireMass * desiredAccel)-tireWorldVel, Color.cyan);
        carRigidBody.AddForceAtPosition(steeringDir * tireMass * desiredAccel, wheels[i].transform.position);
    }

    void AddLongitudinalForce(int i) // In accordance with Asawicki.info
    {
        accelDir = transform.forward;
        Debug.DrawRay(transform.position,accelDir*10,Color.cyan);
        float dragCoefficient = 0.26f; // might be a bit too much
        Vector3 dragForce = -dragCoefficient * carRigidBody.velocity * carRigidBody.velocity.magnitude;
        float resistanceCoefficient = 10f;
        Vector3 rollResistance = -resistanceCoefficient * carRigidBody.velocity;
        Vector3 longitudinalForce = ((driveForce[i] + brakeForce[i]) * accelDir) 
        + dragForce 
        + rollResistance
        ;
        wheelAngularVelocity[i] = wheels[i].transform.InverseTransformDirection(carRigidBody.velocity).z / tireRadius; // We assume there's no wheelspin so the wheel velocity is always rolling at the same velocity as the ground (also should replace car velocity with velocity at contact point)
        Speed = carRigidBody.velocity.magnitude; // might be a bit too much
        //Debug.DrawRay(wheels[i].transform.position,accelDir*4,Color.blue);

        carRigidBody.AddForceAtPosition(longitudinalForce, wheels[i].transform.position);
    }

    void AddLongitudinalForcePacejka(int i) // In accordance with Asawicki.info
    {
        float slipRatio = (wheelAngularVelocity[i]*tireRadius/wheels[i].transform.InverseTransformDirection(carRigidBody.velocity).z - 1)*100;
        float Force = pacejka(slipRatio, 1.65f,i);
        float dragCoefficient = 0.26f; // might be a bit too much
        Vector3 dragForce = -dragCoefficient * carRigidBody.velocity * carRigidBody.velocity.magnitude;
        float resistanceCoefficient = 10f;
        Vector3 rollResistance = -resistanceCoefficient * carRigidBody.velocity;
        Vector3 longitudinalForce = ((Force + brakeForce[i]) * accelDir) 
        + dragForce 
        + rollResistance
        ;
        wheelAngularVelocity[i] = wheels[i].transform.InverseTransformDirection(carRigidBody.velocity).z / tireRadius; // We assume there's no wheelspin so the wheel velocity is always rolling at the same velocity as the ground (also should replace car velocity with velocity at contact point)
        Speed = carRigidBody.velocity.magnitude; // might be a bit too much
        //Debug.DrawRay(wheels[i].transform.position,accelDir*4,Color.blue);
        carRigidBody.AddForceAtPosition(longitudinalForce, wheels[i].transform.position);

    }
    float pacejka(float slip, float t_shape,int i)
    {
        float stiff = 10;
        float peak = 1;
        float curve = 0;

        return Fz[i] * peak * Mathf.Sin(t_shape * Mathf.Atan(stiff * slip - curve * (stiff * slip - Mathf.Atan(stiff * slip))));
    }
    #endregion

    void UpdateWheelMesh(int i)
    {
        carRims = wheels[i].transform.GetChild(0);
        if (isHit[i])
        {
            wheels[i].transform.GetChild(0).localPosition = new Vector3(0,-(hitDistance-tireRadius),0);
        }
        else
        {
            wheels[i].transform.GetChild(0).localPosition = new Vector3(0,-(maxHitDistance-tireRadius),0);
        }
        wheels[i].transform.localRotation = Quaternion.Euler(new Vector3(0, steeringAngle*steeringFactor[i], 0));
        carRims.transform.Rotate(wheelAngularVelocity[i] * Time.fixedDeltaTime*  Mathf.Rad2Deg,0,0);
        
    }
    #region Input

    float SteeringInterp(float steeringInput)
    {
        steeringAngle = Mathf.Lerp(steeringAngle, maxSteeringAngle * steeringInput, 0.04f * (1/Time.deltaTime));
        return steeringAngle;
    }

    public void GetThrottle()
    {
            if (Input.GetAxisRaw("Vertical") == 1)
            {
                Throttle = Mathf.Clamp(Throttle + Time.fixedDeltaTime * 4, 0, 1);
            }
            else
            {
                Throttle = Mathf.Clamp(Throttle + Time.fixedDeltaTime * -10, 0, 1);
            }

            if (Input.GetAxisRaw("Vertical") == -1)
            {
                Brake = Mathf.Lerp(Brake, 1, 8*Time.deltaTime);
            }
            else
            {
                Brake = Mathf.Lerp(Brake, 0, 16*Time.deltaTime);
            }
    }
    #endregion

    #region Engine
        private void engineUnreal()
        {
            float initialTorque = engineCurve.Evaluate(engineRPM) * Throttle;
            dragTorque = engineBrake + (engineRPM*engineDrag);
            if (Mathf.Sign(wheelAngularVelocity[0]) != Mathf.Sign(get_ratio()))
            {   
                dragTorque = 0;
            }
            if (engineRPM >= rpmLimit)
            {
                engineRPM -= 500;
                initialTorque = 0;
            }
            torque_out = initialTorque - dragTorque;
            
            float engineAccel = torque_out / engineMoment;
            engineAngularVelocity += engineAccel * Time.fixedDeltaTime;
            engineAngularVelocity = Mathf.Clamp(engineAngularVelocity, rpmLimitIdle * RPM_2_AV, rpmLimit * RPM_2_AV);
            engineRPM = engineAngularVelocity * AV_2_RPM;


        }
        private void engineUnreal2()
        {
            float initialTorque = engineCurve.Evaluate(engineRPM) * Throttle;
            dragTorque = engineBrake + (engineRPM*engineDrag);
            if (Mathf.Sign(wheelAngularVelocity[0]) != Mathf.Sign(get_ratio()))
            {   
                dragTorque = 0;
            }
            if (engineRPM >= rpmLimit)
            {
                engineRPM -= 500;
                initialTorque = 0;
            }
            torque_out = initialTorque - dragTorque - Tc;
            
            float engineAccel = torque_out / engineMoment;
            engineAngularVelocity += engineAccel * Time.fixedDeltaTime;
            engineAngularVelocity = Mathf.Clamp(engineAngularVelocity, rpmLimitIdle * RPM_2_AV, rpmLimit * RPM_2_AV);
            engineRPM = engineAngularVelocity * AV_2_RPM;


        }
    #endregion

    #region GearBox
    private void GearUP()
    {
        if (currentGear < numOfGears-1 && gearEngaged)
        {
            gearEngaged = false;
            StartCoroutine(ShiftTime(currentGear + 1));
        }
    }

    private IEnumerator ShiftTime(int shiftDirection)
    {
        currentGear = 1;
        yield return new WaitForSeconds(shiftTime);
        currentGear = shiftDirection;
        gearEngaged = true;

    }

    private void GearDOWN()
    {
        if (currentGear > 0 && gearEngaged)
        {
        gearEngaged = false;
        StartCoroutine(ShiftTime(currentGear - 1));
        }
    }

    private float get_ratio()
    {
        return gearRatios[currentGear] * finalDriveGear;
    }
    #endregion

    #region DriveTrain

    private void engageDriveTrain()
    {
        netDrive = Tc * get_ratio()  / 2.0f / tireRadius;

		for(int i = 0; i < 4; i++)
		{
			if(i >= 2)
			{
				driveForce[i] = netDrive; // powered wheels
			}
			else
			{
				driveForce[i] = 0.0f; // not powered wheels
			}

			AddLateralForce(i); 
			AddLongitudinalForce(i); // Apply forces, update angular velocities
            //AddLongitudinalForcePacejka(i);
			UpdateWheelMesh(i);
		}
        float avgSpin = 0;
        for (int w = 2; w < 4; w++)
        {
            avgSpin += wheelAngularVelocity[w] * 0.5f;
        }
        engineRPM = avgSpin * get_ratio() * AV_2_RPM; // update engine rpm at the enc
        engineRPM = Mathf.Clamp(engineRPM, rpmLimitIdle, rpmLimit);
    }
    #endregion

    #region Brakes

    private void calculateBrakes(int i)
    {
       if (i < 2) 
       {
            brakeForce[i] = -Mathf.Sign(wheelAngularVelocity[i])*brakeStrength * Brake * brakeBias;
       }
       else
       {
            brakeForce[i] = -Mathf.Sign(wheelAngularVelocity[i])*brakeStrength * Brake * (1-brakeBias);
       }
    }
    
    #endregion

    #region Clutch
    private float Clutch()
    {
        float w1 = engineAngularVelocity; // Angular velocity of engine
        float T1 = torque_out - dragTorque; // Engine Output Torque
        float I1 = engineMoment; // Engine Inertia
        float T2 = (driveForce[3] + brakeForce[3])*tireRadius + (driveForce[2] + brakeForce[2])*tireRadius; // Drivetrain Reaction Torque
        float I2 = wheelInertia*2/Mathf.Pow(get_ratio(),2); // DriveTrain Inertia powered wheels, I2=(wi1+wi2+ ... + wiN)/(gear ratio^2)
        float avgSpin = 0;
        for (int w = 2; w < 4; w++)
        {
            avgSpin += wheelAngularVelocity[w] * 0.5f;
        }
        float w2 = avgSpin * finalDriveGear; // Drivetrain Angular Velocity



        Tc = (I1*T2 - I2*T1 - I1*I2) * ((w1- w2)/Time.fixedDeltaTime)/(I1+I2);
        return Tc;
    }
    #endregion
}