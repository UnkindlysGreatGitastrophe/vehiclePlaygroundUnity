using System;
using UnityEngine;


public class CarScriptGodot : MonoBehaviour
{
    Rigidbody rb;
    [Header("Wheel Parameters")]
    float wheelMoment; // Wheel's moment of inertia
    public Transform[] wheels; // Contains the transform of the wheels
    public float radius = 0.45f; // Tire Radius (m)
    public float wheelMass = 12; // Usually 12 kg

    public int[] wheelSteer; // Determine which tires steer
    [Header("Suspension")]
    public float springLength = 0.5f; // Max Length (m)
    public RaycastHit[] getHitLength; // Gets length of contact
    public float springStiffness = 100; // Allows absorption of weight, Set to (m*9.81/(4/preferred rest length))
    public float[] compress; //
    float[] prev_compress;
    public float dampValue = 1; // Rates of which we return to rest length
    public float reboundDamp = 1; // 

    [Header("Velocity")]

    public float[] spin; // Wheel Angular Velocity in Radians/s^2


    [Header("Pacejka")]
    public float peak = 1;
    public float stiff = 10;
    public float curve = 0;
    public float x_shape = 1.35f;
    public float z_shape = 1.65f;
    public float lat_peak;
    public float long_peak;

    [Header("Forces")]
    public float [] lat_force; // Lateral Force in Newtons (Kg*m/s^2)
    public float [] long_force; // Longitudinal Force in Newtons (Kg*m/s^2)

    public float [] slip_ratio; // Slip Ratio 
    public float [] slip_angle; // Slip Angle

    public float[] yForce; // Suspension force (Upwards on y axis)

    private bool isHit;
    public float driveTorque; // Newton Meter
    public float brakeTorque; // Newton Meter

    public float net_torque;  // Newton Meter

    public float SteerInput;  // -1 to 1
    private float tanSlipAngle;

    [Header("Ackermann")]
    
    public float max_steer = 34f;
    public float ackermann = 0.15f;

    [Header("Engine")]

    public AnimationCurve engineCurve;
    
    public float engineRPM;
    public float rpm_limit = 6400; // Also found above in the torque curve loop
    public float engineBreak = 10.0f; // Base value for engine braking/drag (eg. 10.0)
    public float engineDrag = 0.03f; // Drag that increases linearly with RPM (eg. 0.03)
    public float torque_out; // Stores the value we get from calling the torque() function above
    public float engine_moment = 0.25f; // Moment of inertia of the engine internals (eg. 0.25)
    private float driveinertia; // engine_moment multiplied by the current gearing
    
    private float dragTorque;

    private float AV_2_RPM = 60/(2* Mathf.PI);
    private float speedo;

    public enum DriveTrainType 
    {
        FWD,
        RWD,
        AWD
    }
    [Header("DriveTrain")]

    public DriveTrainType driveTrainType;
    public int r_diff = 0;
    public float r_split = 0;

    void Start ()
    {
        rb = GetComponent<Rigidbody>();
        prev_compress = new float[wheels.Length];
        compress = new float[wheels.Length];
        getHitLength = new RaycastHit[wheels.Length];
        wheelMoment = 0.5f * wheelMass * radius * radius;
        spin = new float[wheels.Length];
        yForce = new float[wheels.Length];
        lat_force = new float[wheels.Length];
        long_force = new float[wheels.Length];
        slip_angle = new float[wheels.Length];
        slip_ratio = new float[wheels.Length];
        

    }

    void Update()
    {
        SteerInput = Input.GetAxisRaw("Horizontal");
        //driveTorque = Input.GetAxisRaw("Throttle") * 450;
        brakeTorque = Input.GetAxisRaw("Brake") * 3500;
        for (int i = 0; i < wheels.Length; i++)
        {
            Transform rims = wheels[i].transform.GetChild(0).transform;
            rims.Rotate(new Vector3(spin[i] *Mathf.Rad2Deg * Time.deltaTime, 0, 0));


        }
    }
    void FixedUpdate()
    {   

        
        CalculateDriveTorque();
        ApplyTorquesWithDiff();
        dragTorque = engineBreak + engineRPM * engineDrag;
        torque_out = (CalculateEngineTorque() + dragTorque) * Input.GetAxisRaw("Throttle");
        for (int i = 0; i < wheels.Length; i++)
        {
            
            ApplySuspensionForces(i);
            GetLateralForce(i);
            //ApplyTorques(i);
            SteerInput = Input.GetAxis("Horizontal");
            steer(SteerInput, max_steer, ackermann, i);

        }
        
    }

    private void ApplyTorquesWithDiff()
    {
        //freeWheel(); //Debugging...
        engage();
    }

    private void CalculateDriveTorque()
    {
        engineRPM += AV_2_RPM * Time.fixedDeltaTime * (torque_out - dragTorque)/engine_moment;
            if (engineRPM >= rpm_limit)
            {
                torque_out = 0;
                engineRPM -= 500;
            }
    }

    private float CalculateEngineTorque()
    {
        engineRPM = Mathf.Clamp(engineRPM, 0, rpm_limit);
        return engineCurve.Evaluate(engineRPM);
    }

    private void freeWheel()
    {
        float avg_spin = 0;
        for  (int i = 0; i<wheels.Length; i++)
        {
            apply_torque(0,0,brakeTorque,i);
        }
        if (driveTrainType == DriveTrainType.RWD)
        {
            for (int w = 2; w < 4; w++){
                avg_spin += spin[w] * .5f;
            }
        }
        if (driveTrainType == DriveTrainType.FWD)
        {
            for (int w = 0; w < 2; w++){
                avg_spin += spin[w] * .5f;
            }
        }
        if (driveTrainType == DriveTrainType.AWD)
        {
            for (int w = 0; w < wheels.Length; w++){
                avg_spin += spin[w] * .5f;
            }
        }
        speedo = avg_spin * radius;
        
    }

    private void engage()
    {
        float avg_spin = 0;
        float net_Drive = (torque_out - dragTorque)* gear_ratio();
        if (driveTrainType == DriveTrainType.RWD)
        {
            for (int w = 2; w < 4; w++){
                avg_spin += spin[w] * 0.5f;
            }
        }
        if (driveTrainType == DriveTrainType.FWD)
        {
            for (int w = 0; w < 2; w++){
                avg_spin += spin[w] * 0.5f;
            }
        }
        if (driveTrainType == DriveTrainType.AWD)
        {
            for (int w = 0; w < wheels.Length; w++){
                avg_spin += spin[w] * 0.25f;
            }
        }

        if (avg_spin * Mathf.Sign(gear_ratio()) < 0)
        {
            net_Drive += dragTorque * gear_ratio();
        }

        rwd(net_Drive);
        speedo = avg_spin * radius;

        for (int w = 0; w < 2; w++)
        {
            apply_torque(0,0,brakeTorque,w);
        }
        engineRPM = avg_spin * gear_ratio() * AV_2_RPM;
    }

    private float apply_torque(float drive, float driveinertia, float brakeTorque, int i)
    {
            float prev_spin = spin[i];
            net_torque = -long_force[i] * radius;
            net_torque += drive;
            if (Mathf.Abs(spin[i]) < 5 && brakeTorque > Mathf.Abs(net_torque))
            {
                spin[i] = 0;
            }
            else
            {
                net_torque -= brakeTorque * Mathf.Sign(spin[i]);
                spin[i] += Time.fixedDeltaTime * net_torque / (wheelMoment + driveinertia);
            }    
            if (drive * Time.fixedDeltaTime == 0)
            {
                return 0.5f;
            }
            else
            {
                return (spin[i] - prev_spin) * (wheelMoment + driveinertia) / (drive * Time.fixedDeltaTime);
            }
            }

    private void rwd(float net_Drive)
    {
        
        apply_torque(net_Drive * 0.5f, driveinertia, brakeTorque,2);
        apply_torque(net_Drive * 0.5f, driveinertia, brakeTorque,3);

    }
    

    private float gear_ratio()
    {
        return 3.615f;
    }

    

    void ApplySuspensionForces(int i)
    {
        prev_compress[i] = compress[i]; // Store previous value
        isHit = Physics.Raycast(wheels[i].transform.position, -wheels[i].transform.TransformDirection(Vector3.up), out getHitLength[i], springLength); // Cast a ray from the position of the wheel, headed downwards from the ray, at a length of the spring
        if (isHit)
            compress[i] = springLength - getHitLength[i].distance; // 1 is fully compressed
        else
            compress[i] = 0; // Max Length
        yForce[i] = springStiffness * compress[i]; //  What is this in units? Kg*m/s^2?
        if ((compress[i] - prev_compress[i]) >= 0) 
            yForce[i] += dampValue * (compress[i] - prev_compress[i]) / Time.fixedDeltaTime;
        else
        {  
            yForce[i] += reboundDamp * (compress[i] - prev_compress[i]) / Time.fixedDeltaTime;
        }
        if (isHit)
            rb.AddForceAtPosition(yForce[i] * getHitLength[i].normal, getHitLength[i].point);
    }

    void GetLateralForce(int i)
    {
     //   SimpleSlipRatio(i);
        OscillationSlipRatio(i);
        lat_force[i] = -pacejka(slip_angle[i],x_shape, lat_peak, i);
        long_force[i] = pacejka(slip_ratio[i],z_shape, long_peak, i);
        if (slip_ratio[i] == 0 && slip_angle[i] == 0)
        {
            lat_force[i] = 0;
            long_force[i] = 0;  
        }
        else
        {
            lat_force[i] =  lat_force[i] * (Mathf.Abs(slip_angle[i])/Mathf.Sqrt(Mathf.Pow(slip_ratio[i],2) + Mathf.Pow(slip_angle[i],2)));
            long_force[i] = long_force[i] * (Mathf.Abs(slip_ratio[i])/Mathf.Sqrt(Mathf.Pow(slip_ratio[i],2) + Mathf.Pow(slip_angle[i],2)));
        }
        


        if (isHit)
        {
            rb.AddForceAtPosition(wheels[i].right * lat_force[i], getHitLength[i].point + radius*transform.up);
            //rb.AddForceAtPosition(wheels[i].forward * long_force[i], getHitLength[i].point + radius*transform.up);
            //Debug.DrawRay(getHitLength[i].point + radius*transform.up, wheels[i].right * lat_force[i],Color.red);
            //Debug.DrawRay(getHitLength[i].point + radius*transform.up, wheels[i].forward * long_force[i],Color.blue);

        }


    }

    void ApplyTorques(int i)
    {
            net_torque = -long_force[i] * radius;
            net_torque += driveTorque;
            if (Mathf.Abs(spin[i]) < 5 && brakeTorque > Mathf.Abs(net_torque))
            {
                spin[i] = 0;
            }
            else
            {
                net_torque -= brakeTorque * Mathf.Sign(spin[i]);
                spin[i] += Time.fixedDeltaTime * net_torque / wheelMoment;
            }
    }

    void SimpleSlipRatio(int i)
    {
        Vector3 localVelocity = wheels[i].transform.InverseTransformDirection(rb.GetPointVelocity(wheels[i].position)); // M/S
        float z_vel = localVelocity.z; // M/S
        //Debug.Log(z_vel);
        Vector2 planar_vect = new Vector2(localVelocity.x, localVelocity.y).normalized; // Vector where x = longitudinal speed, and y = lateral speed, m/s for both
        //slip_angle[i] = Mathf.Atan(planar_vect.y/Mathf.Max(Mathf.Abs(planar_vect.x),1)); // Suggested Version
        slip_angle[i] = Mathf.Asin(Mathf.Clamp(planar_vect.x,-1,1)); // Tutorial version
        slip_ratio[i] = 0;
        if (z_vel != 0)
            slip_ratio[i] = (spin[i] * radius - z_vel)/Mathf.Abs(z_vel); // Tutorial Version  rad/s * m  - m/s / m/s?
            //slip_ratio[i] = spin[i] * radius/z_vel - 1; // Suggested Version
            

        lat_force[i] = -pacejka(slip_angle[i],x_shape, lat_peak, i);
        long_force[i] = pacejka(slip_ratio[i],z_shape, long_peak, i);
    }

    void OscillationSlipRatio(int i)
    {
        float maxSlipRatio = 1.5f;
        float maxSlipAngle = 90f;

        float wheelVelocity = spin[i] * radius;
        Vector3 localVelocity = wheels[i].transform.InverseTransformDirection(rb.GetPointVelocity(wheels[i].position)); // M/S
        float longitudinalVelocity = localVelocity.z;
        float lateralVelocity = localVelocity.x;
        float absLongVelo = Mathf.Abs(longitudinalVelocity);
        float maxDampedLongVelo = Mathf.Max(absLongVelo, 5f);
        float tau = 0.02f; // Oscillation Perido
        float factor = Mathf.Abs(longitudinalVelocity * 4f) * tau;
        if (factor < 1) factor = 1;
        float relLongFactor = 0.4f * factor;
        float relLatFactor = 0.6f * factor;
        float deltaSlipRatio = (wheelVelocity - longitudinalVelocity) - maxDampedLongVelo * slip_ratio[i];
        deltaSlipRatio /= relLongFactor;
        slip_ratio[i] += deltaSlipRatio * Time.fixedDeltaTime;

        slip_ratio[i] = Mathf.Clamp(slip_ratio[i], -maxSlipRatio, maxSlipRatio);

        float deltaSlipAngle = lateralVelocity - maxDampedLongVelo * tanSlipAngle;
        deltaSlipAngle /= relLatFactor;

        tanSlipAngle += deltaSlipAngle * Time.fixedDeltaTime;

        slip_angle[i] = Mathf.Atan(tanSlipAngle);
        slip_angle[i] = Mathf.Clamp(slip_angle[i], -maxSlipAngle*Mathf.Deg2Rad, maxSlipAngle*Mathf.Deg2Rad);

    }

    void steer(float input, float max_steer, float ackermann, int i)
    {
        //Debug.Log(wheelSteer[i] * max_steer * input);
        wheels[i].transform.localRotation = Quaternion.Euler( 0,wheelSteer[i] * max_steer * input,0);
    }

    float pacejka(float slip, float t_shape, float peak, int i)
    {
        return yForce[i] * peak * Mathf.Sin(t_shape * Mathf.Atan(stiff * slip - curve * (stiff * slip - Mathf.Atan(stiff * slip))));

    }
}
