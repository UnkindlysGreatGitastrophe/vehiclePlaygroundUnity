using BezierSolution;
using UnityEngine;
using Baracuda.Monitoring;
using System.Collections.Generic;
using System.Collections;

public class AICarController : MonoBehaviour
{

    [Header("References")]
    public CarObj car;
    public BezierSpline[] lineToFollow;

    float normalizedT;
    public float currentOffset = 10;
    private float maxCornerSpeed;
    private float maxBrakeDistance;
    private float lookAheadDistance;
    public float minLookAheadDist = 5;
    public float maxLookAheadDist = 50;
    public int currentSpline;
    public bool findNearestSpline;

    public float rangeThreshold = 50;
    public List<BezierSpline> foundBezierSplinePaths;

    [Header("AirTime Controller")]
    public float timeToFlip;

    private RaycastHit hitInfo;
    public float distanceBelow;
    [Monitor] private Vector3 angleDiffs;
    [Monitor] private Quaternion lastRotation;
    [Monitor] private Vector3 totalRotation;
    public Vector3 angularDisplacement;
    [Monitor] public Vector3 stuntsCount;
    public float offset;
    public float rotationThresholdoffset = 75;
    private Coroutine myRunningCoroutine;
    public Vector3 timeToStop;
    public bool randomDirectionChoice;
    public int chosenDirection;
    public float stuntTime;
    public Vector3 maxAngularVelocity;
    public float estimatedTimeToFlip = 1.25f;

    // Start is called before the first frame update
    void Start()
    {
        this.StartMonitoring();

        if (findNearestSpline)
        {
            float closestSpline = Mathf.Infinity;
            for (int i = 0; i < lineToFollow.Length; i++)
            {
                Vector3 closestPointToCar = lineToFollow[i].FindNearestPointTo(car.transform.position, 95);
                float newDistance = Vector3.Distance(closestPointToCar, car.transform.position);
                if (newDistance < closestSpline)
                {
                    closestSpline = newDistance;
                    currentSpline = i;
                }
            }
        }

        for (int i = 0; i < 3; i++)
        {
            maxAngularVelocity[i] = car.airRotationAmount / (car.rb.inertiaTensor[i] * car.rb.angularDrag);
            timeToFlip = Mathf.PI * 2 / maxAngularVelocity[i];
            float d = maxAngularVelocity[i] * 0.93f / 0.8f;
            float a = 1f - car.rb.angularDrag * Time.fixedDeltaTime;
            timeToStop[i] = -Time.fixedDeltaTime * (Mathf.Floor(Mathf.Log(d) / Mathf.Log(a)) - 2);
            Debug.Log(timeToStop);
            angularDisplacement[i] = 0.5f * maxAngularVelocity[i] * timeToStop[i];
            Debug.Log(angularDisplacement);
        }

    }

    // Update is called once per frame
    void Update()
    {


        /*         if (Input.GetKeyDown(KeyCode.R))
                {
                    car.transform.position = nearestPointToSpline;
                } */

        NormalDriving2();
        lineToFollow[currentSpline].FindNearestPointTo(car.transform.position, out float closestT, 95);
        if (!lineToFollow[currentSpline].loop && closestT > 0.99f && findNearestSpline)
        {
            SearchForAlternateSpline();
        }
    }

    void FixedUpdate()
    {
        if (car.isCarMidAir())
        {
            TrackStunts();
            lastRotation = car.rb.rotation;
            getDistanceFromGround();
            stuntTime = getTimeToHitGround();
            // List<int> stuntsOrientation = new List<int>();
            // for (int i = 0; i < 3; i++)
            // {
            //     if (stuntTime > timeToFlip + timeToStop[i])
            //     {
            //         stuntsOrientation.Add(i);
            //     }

            //     // This formula comes from solving the differential equation for deceleration.
            //     // It assumes no external torque is applied during this time.
            //     TrackStunts();
            // }
            // if (stuntsOrientation.Count > 0)
            // {
            if (myRunningCoroutine == null && stuntTime > estimatedTimeToFlip)
            {

                //offset = Mathf.DeltaAngle(0, lastRotation.eulerAngles.x);
                totalRotation.x = Mathf.DeltaAngle(0, lastRotation.eulerAngles.x);
                myRunningCoroutine = StartCoroutine(ProvideSpinTest2(stuntsCount, chosenDirection));
            }
            // }
            // stuntsOrientation.Clear();
        }
        else
        {
            if (randomDirectionChoice)
                chosenDirection = UnityEngine.Random.Range(0, 3);
            totalRotation = Vector3.zero;
        }


    }


    void NormalDriving2()
    {

        // Cornering Speed Section

        maxBrakeDistance = calculateBrakeDistance(); // Get Maximum Brake Distance to the corner speed

        lookAheadDistance = Mathf.Clamp(maxBrakeDistance - (car.BrakeInput * 5 * Time.deltaTime), minLookAheadDist, maxLookAheadDist);

        Vector3 closestPointToLookAhead = lineToFollow[currentSpline].FindNearestPointTo(car.transform.position + (car.transform.forward * lookAheadDistance), out normalizedT, 95);

        Debug.DrawLine(car.transform.position, closestPointToLookAhead, Color.yellow);

        maxCornerSpeed = TestCornerSpeed2(normalizedT);

        //print("Max Corner Speed: " + maxCornerSpeed + " km/h");


        // Steering Section

        float steeringLookAheadDistance = ClutchObj.Remap(car.carSpeed, 5, 200, 2, 50);

        Vector3 closestPointSteerToLookAhead = lineToFollow[currentSpline].FindNearestPointTo(car.transform.position + (car.transform.forward * steeringLookAheadDistance), 95);

        Debug.DrawLine(car.transform.position, closestPointSteerToLookAhead, Color.black);


        Vector3 worldDirection = closestPointSteerToLookAhead - car.transform.position;

        Vector3 localDirection = transform.InverseTransformDirection(worldDirection);

        float angleXZ = Mathf.Atan2(localDirection.x, localDirection.z) * Mathf.Rad2Deg;

        //print("Steering Angle: " + angleXZ + " Degrees");

        car.steeringInput = ClutchObj.Remap(angleXZ, -car.clampedSteeringAngle, car.clampedSteeringAngle, -1, 1);



        // Throttle/Brake Section:

        float velocityDiff = car.carSpeed - maxCornerSpeed;
        if (velocityDiff > 0.0) //braking case
        {
            car.BrakeInput = ClutchObj.Remap(velocityDiff, 1, 5, 0.0f, 1.0f);
            car.Throttle = ClutchObj.Remap(velocityDiff, 5, 25, 1f, 0f);
        }
        else
        {
            car.Throttle = 1;
            car.BrakeInput = 0;

        }

    }

    float calculateBrakeDistance(float desiredSpeed = 0)
    {
        float maxBrakeForce = car.maxBrakeTorque / car.GetAVGTireRadius(); // 
        float maxBrakeDeceleration = maxBrakeForce / car.rb.mass;
        // Δx = (v_f² - v_i²) / 2a
        float distance = (Mathf.Pow(desiredSpeed / 3.6f, 2) - Mathf.Pow(car.carSpeed / 3.6f, 2)) / (2 * maxBrakeDeceleration);

        return Mathf.Abs(distance);
    }

    float TestCornerSpeed2(float firstPointValuet)
    {
        Vector3 point1 = lineToFollow[currentSpline].GetPoint(firstPointValuet); // B
        Vector3 point2 = lineToFollow[currentSpline].GetPoint(firstPointValuet + (currentOffset / lineToFollow[currentSpline].length)); // A
        Vector3 point3 = lineToFollow[currentSpline].GetPoint(firstPointValuet + 2 * (currentOffset / lineToFollow[currentSpline].length)); // C

        // Vector3 point1 = lineToFollow[0].GetPoint(firstPointValuet - 2 * (currentOffset / lineToFollow[0].length)); // B
        // Vector3 point2 = lineToFollow[0].GetPoint(firstPointValuet); // A
        // Vector3 point3 = lineToFollow[0].GetPoint(firstPointValuet + 2 * (currentOffset / lineToFollow[0].length)); // C

        Debug.DrawLine(car.transform.position, point2, Color.red);
        Debug.DrawLine(car.transform.position, point3, Color.red);

        // Debug.DrawLine(car.transform.position, point1, Color.red);
        // Debug.DrawLine(car.transform.position, point3, Color.red);

        float a = Vector3.Distance(point1, point3); // B -> C
        float b = Vector3.Distance(point2, point3); // A -> C
        float c = Vector3.Distance(point1, point2); // B -> A

        float A = Mathf.Acos((Mathf.Pow(b, 2) + Mathf.Pow(c, 2) - Mathf.Pow(a, 2)) / (2 * b * c)); // If this return NAN, that means that we are on a straightaway, radius is irrevelant, so we assume there is no limits on top speed.
        float radius = a / (2 * Mathf.Sin(Mathf.PI - A));
        float maxCornerSpeedMS = Mathf.Sqrt(radius * car.GetAVGTireGrip() * Mathf.Abs(Physics.gravity.y));
        return maxCornerSpeedMS * 3.6f;

    }

    void SearchForAlternateSpline()
    {

        float closestSpline = Mathf.Infinity;
        int splineBeforeSwitch = currentSpline;
        for (int i = 0; i < lineToFollow.Length; i++)
        {
            Vector3 endPointofSpline = lineToFollow[splineBeforeSwitch].GetPoint(1);
            float newDistance = Vector3.Distance(endPointofSpline, lineToFollow[i].GetPoint(0));
            if (newDistance < rangeThreshold && newDistance < closestSpline && splineBeforeSwitch != i)
            {
                foundBezierSplinePaths.Add(lineToFollow[i]);
            }
        }

        if (foundBezierSplinePaths.Count > 0)
        {
            currentSpline = System.Array.IndexOf(lineToFollow, foundBezierSplinePaths[UnityEngine.Random.Range(0, foundBezierSplinePaths.Count)]);
            foundBezierSplinePaths.Clear();
            //findNearestSpline = false;
        }
    }
    

    

    // Call this when you initiate the flip (e.g., applying torque)
    void getDistanceFromGround()
    {
        if (Physics.Raycast(transform.position, Vector3.down, out hitInfo))
        {
            // If the ray hits something, get the distance
            distanceBelow = hitInfo.distance;
            //Debug.Log($"Distance below point: {distanceBelow} units.");

            // Optional: Draw a debug ray to visualize
            //Debug.DrawRay(transform.position, Vector3.down * distanceBelow, Color.green);
        }
        else
        {
            // If the ray doesn't hit anything within maxRayLength
            //Debug.Log("No collider found below the point within the specified range.");
            //Debug.DrawRay(transform.position, Vector3.down * 50f, Color.red);
        }

    }

    float getTimeToHitGround()
    {
        // Coefficients for the quadratic equation: at^2 + bt + c = 0
        float a = 0.5f * Physics.gravity.y;
        float b = car.rb.velocity.y;
        float c = distanceBelow;
        // Calculate the discriminant
        float discriminant = (b * b) - (4 * a * c);

        if (discriminant >= 0)
        {
            // Two possible solutions for time
            float time1 = (-b + Mathf.Sqrt(discriminant)) / (2 * a);
            float time2 = (-b - Mathf.Sqrt(discriminant)) / (2 * a);

            // Choose the positive time that makes sense (e.g., if starting above ground and falling)
            float timeToHitGround = Mathf.Max(time1, time2);

            //Debug.Log($"Time to hit the ground: {timeToHitGround} seconds");
            return timeToHitGround;

        }
        else
        {
            //Debug.Log("Object may not hit the ground (e.g., if initial velocity is too high upwards and no ground is below).");
            return 0;
        }
    }


    void TrackStunts() // Tracks the various stunts a player may perform in the air.
    {


        angleDiffs = Vector3.zero; // The difference between the current and last rotation in Euler Angles
        Quaternion difference = Quaternion.Inverse(car.rb.rotation) * lastRotation; // Take the difference between current and last rotation (Result is in Quaternions)
        lastRotation = car.rb.rotation; // Update last rotation

        angleDiffs.x = Mathf.DeltaAngle(0, difference.eulerAngles.x); // Calculates the shortest distance between 0 degrees and the euler angle of the difference in all 3 dimensions 
        angleDiffs.y = Mathf.DeltaAngle(0, difference.eulerAngles.y); // (This is used to prevent getting -270 degrees instead of 90 degrees for example)
        angleDiffs.z = Mathf.DeltaAngle(0, difference.eulerAngles.z);

        totalRotation.x += angleDiffs.x; // Add the difference to the total rotation
        totalRotation.y += angleDiffs.y; // Add the difference to the total rotation
        totalRotation.z += angleDiffs.z; // Add the difference to the total rotation

        // Increase stunt count for when a full revolution is complete.

        if (totalRotation.x < (-360 + offset + rotationThresholdoffset))
        {
            totalRotation.x = 0;
            stuntsCount[0]++;
        }
        else if (totalRotation.x > (360 - offset - rotationThresholdoffset))
        {
            totalRotation.x = 0;
            stuntsCount[0]++;

        }

        if (totalRotation.y < -360)
        {
            totalRotation.y = 0;
            stuntsCount[1]++;
        }
        else if (totalRotation.y > 360 )
        {
            totalRotation.y = 0;
            stuntsCount[1]++;

        }

        if (totalRotation.z < (-360 + offset + rotationThresholdoffset))
        {
            totalRotation.z = 0;
            stuntsCount[2]++;
        }
        else if (totalRotation.z > (360 - offset - rotationThresholdoffset) )
        {
            totalRotation.z = 0;
            stuntsCount[2]++;

        }
    }


    private IEnumerator ProvideSpinTest2(Vector3 prevStunts, int stuntType)
    {
        float initialTime = Time.time;
        while (true)
        {
            if (stuntType == 0)
            {
                car.rb.AddTorque(car.airRotationAmount * car.rb.transform.right, ForceMode.VelocityChange);
                if (stuntsCount[0] != prevStunts[0])
                {
                    myRunningCoroutine = null;
                    Debug.Log("Time to complete stunt: " + (Time.time - initialTime));
                    yield break;

                }
                yield return null;
            }

            if (stuntType == 1)
            {
                car.rb.AddTorque(car.airRotationAmount * car.rb.transform.up, ForceMode.VelocityChange);
                if (stuntsCount[1] != prevStunts[1])
                {
                    myRunningCoroutine = null;
                    Debug.Log("Time to complete stunt: " + (Time.time - initialTime));
                    yield break;
                }
                yield return null;
            }

            if (stuntType == 2)
            {
                car.rb.AddRelativeTorque(0f, 0f, car.airRotationAmount, ForceMode.VelocityChange);
                if (stuntsCount[2] != prevStunts[2])
                {
                    myRunningCoroutine = null;
                    Debug.Log("Time to complete stunt: " + (Time.time - initialTime));
                    yield break;
                }
                yield return null;
            }
            if (!car.isCarMidAir())
            {
                myRunningCoroutine = null;
                yield break;
            }
            yield return null;

        }

    }

    // IEnumerator TimeToStop()
    // {

    //     float initialTime = Time.time;
    //     while (rb.angularVelocity.x > 0.80f)
    //     {
    //         yield return null;
    //     }
    //     Debug.Log("Time to reach 0.8 rad/s: " + (Time.time - initialTime));
    // }

    // IEnumerator TimeToAccel()
    // {

    //     float initialTime = Time.time;
    //     while (rb.angularVelocity.x < (maxAngularVelocity - (1-rb.angularDrag*Time.fixedDeltaTime)))
    //     {
    //         yield return null;
    //     }
    //     Debug.Log("Time to reach "+ (maxAngularVelocity - (1-rb.angularDrag*Time.fixedDeltaTime)) + ": " + (Time.time - initialTime));
    // }



}
