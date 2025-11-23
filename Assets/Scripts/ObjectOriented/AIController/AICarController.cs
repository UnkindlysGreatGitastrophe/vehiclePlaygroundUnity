using BezierSolution;
using UnityEngine;
using Baracuda.Monitoring;
using System.Collections.Generic;
using System.Collections;
using System;
using Unity.VisualScripting.FullSerializer;


[DefaultExecutionOrder(1)]

public class AICarController : MonoBehaviour
{
    [Header("References")]
    public CarObj car;
    public BezierSpline[] lineToFollow;

    float normalizedT;
    public float currentOffset = 10;
    private float maxCornerSpeed;
    private float steeringLookAheadDistance;
    private float maxBrakeDistance;
    private float lookAheadDistance;
    private Vector3 closestPointToLookAhead;
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
    private Vector3 angleDiffs;
    private Quaternion lastRotation;
    private Vector3 totalRotation;
    public Vector3 angularDisplacement;
    public Vector3 stuntsCount;
    public float offset;
    public float rotationThresholdoffset = 75;
    private Coroutine myRunningCoroutine;
    public Vector3 timeToStop;
    public bool randomDirectionChoice;
    public int chosenDirection;
    public float stuntTime;
    public Vector3 maxAngularVelocity;
    public float estimatedTimeToFlip = 1.25f;
    [Monitor] private float velocityDiff;
    public float nitroDeactivationThreshold;
    public float maxNitroDeactivationPerLevel;

    [Header("PID")]
    public float proportionalGain;
    private Vector3 errorLast;
    public float derivativeGain;
    private Vector3 integrationStored;
    public float integralGain;

    [Header("Context Steering")]

    private float resultAngle;
    float[] interest;
    float[] danger;
    bool[] rayHits;
    [Monitor] float[] angles;
    RaycastHit[] raycastHits;
    private bool tiebreakerMode;
    private float recordedSumOfDangers;
    private int steerDirection;
    private float angleTest;
    private float timeUntilReverse;
    private bool isReversing;
    private float reverseTime;

    [Header("Laptime Tracking")]
    public int minimumSplinesTraveresed;
    public List<BezierSpline> uniqueSplinesTraveresed;
    public List<BezierSpline> requiredSplines;
    public int lapsCovered = 0;

    private void OnValidate()
    {
        maxNitroDeactivationPerLevel = 1 / (car.nitroSystem.maxOverBoostPenalty - 1);

    }

    // Start is called before the first frame update
    void Start()
    {
        //this.StartMonitoring();
        interest = new float[4];
        raycastHits = new RaycastHit[4];
        rayHits = new bool[4];
        danger = new float[4];
        angles = new float[4];
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
            //Debug.Log(timeToStop);
            angularDisplacement[i] = 0.5f * maxAngularVelocity[i] * timeToStop[i];
            //Debug.Log(angularDisplacement);
        }

    }

    // Update is called once per frame
    void Update()
    {


        /*         if (Input.GetKeyDown(KeyCode.R))
                {
                    car.transform.position = nearestPointToSpline;
                } */

        lineToFollow[currentSpline].FindNearestPointTo(car.transform.position, out float closestT, 95);
        if (!lineToFollow[currentSpline].loop && closestT > 0.99f && findNearestSpline)
        {
            if (!uniqueSplinesTraveresed.Contains(lineToFollow[currentSpline]))
            {
                uniqueSplinesTraveresed.Add(lineToFollow[currentSpline]);
            }
            SearchForAlternateSpline();
        }

        if (car.isAIPowered)
        {
            NormalDriving2();
        }

        if (uniqueSplinesTraveresed.Count >= minimumSplinesTraveresed)
        {
            for (int i = 0; i < requiredSplines.Count; i++)
            {
                if (!uniqueSplinesTraveresed.Contains(requiredSplines[i]))
                {
                    break;
                }
            }
            lapsCovered++;
            uniqueSplinesTraveresed.Clear();
        }



    }

    void FixedUpdate()
    {
        if (car.isAIPowered)
        {
            AINitroBehaviour();
            if (car.isCarMidAir())
            {
                TrackStunts();
                lastRotation = car.rb.rotation;
                getDistanceFromGround();
                stuntTime = getTimeToHitGround();
                car.PIDengaged = false;

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
                else if (myRunningCoroutine == null && car.isCarMidAir() && stuntTime < estimatedTimeToFlip)
                {
                    engageStabilization();
                }
            }
            else
            {
                if (randomDirectionChoice)
                    chosenDirection = UnityEngine.Random.Range(0, 3);
                totalRotation = Vector3.zero;
                if (!car.PIDengaged)
                {
                    for (int i = 0; i < car.wheels.Length; i++)
                    {
                        float previousGrip = car.wheels[i].tireGripFactor;
                        car.wheels[i].tireGripFactor = 0;
                        StartCoroutine(car.wheels[i].disengageGrip(previousGrip));
                        car.Throttle = 0;

                    }
                }
                car.PIDengaged = true;


            }

        }


    }



    void NormalDriving2()
    {

        // Cornering Speed Section
        maxBrakeDistance = calculateBrakeDistance(); // Get Maximum Brake Distance to a standstill
        lookAheadDistance = Mathf.Clamp(maxBrakeDistance - (car.BrakeInput * 5 * Time.deltaTime), minLookAheadDist, maxLookAheadDist); // Add a bit of an offset when brakes applied 
        closestPointToLookAhead = lineToFollow[currentSpline].FindNearestPointTo(car.transform.position, out normalizedT, 95); // Get closest point, no lookahead distance applied yet.
        normalizedT = Mathf.Clamp01(normalizedT + (lookAheadDistance / lineToFollow[currentSpline].length)); // Get offset point here, add lookahead distance here
        maxCornerSpeed = TestCornerSpeed2(normalizedT); // Get Corner Speed using 3 points here
                                                        //print("Max Corner Speed: " + maxCornerSpeed + " km/h");


        // Throttle/Brake Section:

        velocityDiff = car.carSpeed - maxCornerSpeed;
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

        // Steering Section
        steeringLookAheadDistance = ClutchObj.Remap(car.carSpeed, 5, 200, 2, 50);
        float contextLookAheadDistance = ClutchObj.Remap(car.carSpeed, 5, 200, 2, 100);
        Vector3 closestPointSteerToLookAhead = lineToFollow[currentSpline].FindNearestPointTo(car.transform.position, out normalizedT, 95); // Get Nearest point without offset
        normalizedT = Mathf.Clamp01(normalizedT + (steeringLookAheadDistance / lineToFollow[currentSpline].length)); // Add lookahead distance to steering
        if (checkForObstructionAtPoint(lineToFollow[currentSpline].GetPoint(normalizedT), contextLookAheadDistance))
        {
            closestPointSteerToLookAhead = ContextSteering2();
        }
        else
        {
            closestPointSteerToLookAhead = lineToFollow[currentSpline].GetPoint(normalizedT); // Get actual point with lookahead distance
        }
        //closestPointSteerToLookAhead = ContextSteering2();
        Vector3 worldDirection = closestPointSteerToLookAhead - car.transform.position;
        Vector3 localDirection = transform.InverseTransformDirection(worldDirection);


        float angleXZ = Mathf.Atan2(localDirection.x, localDirection.z) * Mathf.Rad2Deg;
        angleTest = Mathf.Lerp(angleTest, angleXZ, 10 * Time.deltaTime);
        print("Steering Angle: " + angleXZ + " Degrees");
        car.steeringInput = Mathf.Sign(car.carSpeed) * ClutchObj.Remap(angleTest, -car.clampedSteeringAngle, car.clampedSteeringAngle, -1, 1);
        //car.steeringInput = Mathf.Lerp(car.steeringInput, ClutchObj.Remap(resultSteer, -car.clampedSteeringAngle, car.clampedSteeringAngle, -1, 1),Time.deltaTime*2);


        if (car.carSpeed < 10 && !car.isCarMidAir())
        {
            timeUntilReverse += Time.deltaTime;
            if (timeUntilReverse > 3f)
                isReversing = true;
            if (isReversing)
                AIReverseMode();
        }
        else
        {
            timeUntilReverse = 0;
        }

        float avgRearSlipAngle = (car.wheels[2].slipAngle + car.wheels[3].slipAngle) / 2f;
        float avgFrontSlipAngle = (car.wheels[0].slipAngle + car.wheels[1].slipAngle) / 2f;

        if (Mathf.Abs(avgRearSlipAngle) - Mathf.Abs(avgFrontSlipAngle) > 1)
        {
            car.Throttle = 0;
            car.eBrakeInput = 1;
        }
        else
        {
            car.eBrakeInput = 0;
        }


    }



    private bool checkForObstructionAtPoint(Vector3 pointOfInterest, float steeringLookAheadDistance)
    {
        Vector3 rayOrigin = car.transform.position;
        Vector3 rayDirection = pointOfInterest - car.transform.position; // Calculate direction and length
        Debug.DrawRay(rayOrigin, rayDirection.normalized * Vector3.Distance(transform.position, pointOfInterest), Color.yellow);
        Debug.DrawRay(rayOrigin, transform.forward * steeringLookAheadDistance, Color.red);
        steeringLookAheadDistance = Mathf.Max(steeringLookAheadDistance, 5);
        RaycastHit R1;
        RaycastHit R2;
        if (Physics.Raycast(transform.position, rayDirection.normalized, out R1, Vector3.Distance(transform.position, pointOfInterest), ~LayerMask.GetMask("Player", "Ignore Raycast")))
        {

            if (R1.collider != null)
            {
                if (Mathf.Abs(R1.normal.y) < 0.1f) // Small threshold to account for slight angles
                {
                    return true;
                }
            }

        }
        if (Physics.Raycast(transform.position, transform.forward, out R2, steeringLookAheadDistance, ~LayerMask.GetMask("Player", "Ignore Raycast")))
        {

            if (R2.collider != null)
            {
                if (Mathf.Abs(R2.normal.y) < 0.1f) // Small threshold to account for slight angles
                {
                    return true;
                }
            }
        }
        return false;
    }

    private Vector3 ContextSteering2()
    {
        // Perform the Raycast
        float contextLookAheadDistance = ClutchObj.Remap(car.carSpeed, 5, 200, 2, 100);
        Vector3 closestPointSteerToLookAhead = lineToFollow[currentSpline].FindNearestPointTo(car.transform.position, out normalizedT, 95); // Get Nearest point without offset
        normalizedT = Mathf.Clamp01(normalizedT + (contextLookAheadDistance / lineToFollow[currentSpline].length)); // Add lookahead distance to steering
        closestPointSteerToLookAhead = lineToFollow[currentSpline].GetPoint(normalizedT); // Get actual point with lookahead distance
        float sumOfDangers = 0;
        Vector3 rayDirection;

        for (int i = 0; i < 4; i++)
        {
            if (i > 1)
            {
                angles[i] = (((float)i - 1) / 2) * car.clampedSteeringAngle;
                rayDirection = Quaternion.Euler(0, (((float)i - 1) / 2) * car.clampedSteeringAngle / 2, 0) * transform.forward;


            }
            else
            {
                angles[i] = (((float)i - 2) / 2) * car.clampedSteeringAngle;
                rayDirection = Quaternion.Euler(0, (((float)i - 2) / 2) * car.clampedSteeringAngle / 2, 0) * transform.forward;

            }
            rayHits[i] = Physics.Raycast(transform.position, rayDirection, out raycastHits[i], contextLookAheadDistance, ~(1 << LayerMask.NameToLayer("Player")));
            interest[i] = Mathf.Max(0f, Vector3.Dot(rayDirection, (closestPointSteerToLookAhead - car.transform.position).normalized));
            if (raycastHits[i].collider != null)
            {
                if (Mathf.Abs(raycastHits[i].normal.y) < 0.1f) // Small threshold to account for slight angles
                {

                    // This is likely a wall
                    //Debug.Log(String.Format("Raycast #{0} Hit a wall.", i));
                    Debug.DrawRay(transform.position, rayDirection * raycastHits[i].distance, Color.red);
                    danger[i] = 1;
                    //interest[i] = 0f;
                    sumOfDangers += 1;

                }
            }
            else
            {
                Debug.DrawRay(transform.position, rayDirection * contextLookAheadDistance);
                danger[i] = 0;
            }
        }

        resultAngle = 0;
        float leftSideValDanger = getLeftSideAngleValues(true);
        float rightSideValDanger = getRightSideAngleValues(true);

        if (sumOfDangers > 1)
            car.BrakeInput = ClutchObj.Remap(getAverageObstacleDistance(contextLookAheadDistance), contextLookAheadDistance * 0.25f, contextLookAheadDistance, 1, 0);

        if (tiebreakerMode == true && Mathf.Abs(resultAngle) < 1 && sumOfDangers > 1)
        {
            if (steerDirection == 1)
            {
                resultAngle = (angles[0] * interest[0]) + (angles[1] * interest[1]);
                //car.BrakeInput = 1;
            }
            else
            {
                resultAngle = (angles[2] * interest[2]) + (angles[3] * interest[3]);
                //car.BrakeInput = 1;

            }
        }

        else if (tiebreakerMode == false && Mathf.Abs(leftSideValDanger - rightSideValDanger) < 1 && sumOfDangers > 1)
        {
            float leftSideVal = getLeftSideAngleValues();
            float rightSideVal = getRightSideAngleValues();
            if (leftSideVal > rightSideVal)
            {
                resultAngle = leftSideVal;
                steerDirection = 0;
            }
            else
            {
                resultAngle = rightSideVal;
                steerDirection = 1;
            }
            tiebreakerMode = true;
            recordedSumOfDangers = sumOfDangers;
        }
        else
        {
            for (int i = 0; i < 4; i++)
            {
                resultAngle += angles[i] * (interest[i] * (1 - danger[i]));
            }
            tiebreakerMode = false;
        }







        /**********************************************************************
        for (int i = 0; i < 4; i++)
        {
            resultAngle += angles[i] * (interest[i] * (1-danger[i]));
        }

        if (tiebreakerMode == true && Mathf.Abs(resultAngle) < 1 && sumOfDangers > 1)
        {
            if (steerDirection == 1)
            {
                resultAngle = (angles[0] * interest[0]) + (angles[1] * interest[1]);
                car.BrakeInput = 1;
            }
            else
            {
                resultAngle = (angles[2] * interest[2]) + (angles[3] * interest[3]);
                car.BrakeInput = 1;

            }
        }

        if (tiebreakerMode == false && danger[2] == 1 && sumOfDangers > 1)
        {
            if (Mathf.Abs(angles[0] * interest[0] + angles[1] * interest[1]) > Mathf.Abs(angles[2] * interest[2] + angles[3] * interest[3]))
            {
                resultAngle = (angles[0] * interest[0]) + (angles[1] * interest[1]);
                steerDirection = 0;
            }
            else
            {
                resultAngle = (angles[2] * interest[2]) + (angles[3] * interest[3]);
                steerDirection = 1;
            }
            tiebreakerMode = true;
            recordedSumOfDangers = sumOfDangers;
        }
        if (sumOfDangers < recordedSumOfDangers)
        {
            tiebreakerMode = false;

        }
        */

        resultAngle = Mathf.Clamp(resultAngle, -car.clampedSteeringAngle, car.clampedSteeringAngle);
        Vector3 resultDirection = Quaternion.Euler(0, resultAngle, 0) * transform.forward;
        //Debug.Log("Desired Direction is: " + resultAngle + " Degrees");
        //Debug.DrawRay(transform.position, resultDirection, Color.black);

        return transform.position + resultDirection * 40f;
    }

    private void AIShiftToForwards()
    {
        if (car.gearBox.currentGear < 1)
            car.gearBox.GearToFirst();

    }

    private void AIReverseMode()
    {
        if (reverseTime < 3)
        {
            car.steeringInput = 0;
            if (car.gearBox.currentGear > 0)
                car.gearBox.GearREVERSE();
            car.Throttle = 1;
            reverseTime += Time.deltaTime;


        }
        else
        {
            isReversing = false;
            reverseTime = 0;
            timeUntilReverse = 0;
            AIShiftToForwards();
        }
        // do below

        // else
        // isReversing = false;
    }

    float getLeftSideAngleValues(bool considerDanger = false)
    {
        float leftSideAngle = 0;
        for (int i = 0; i < raycastHits.Length / 2; i++)
        {
            if (considerDanger)
            {
                leftSideAngle += angles[i] * (interest[i] * (1 - danger[i]));
            }
            else
            {
                leftSideAngle += angles[i] * (interest[i] * (1 - danger[i]));
            }
        }
        return leftSideAngle;
    }

    float getRightSideAngleValues(bool considerDanger = false)
    {
        float rightSideAngle = 0;
        for (int i = raycastHits.Length / 2; i < raycastHits.Length; i++)
        {
            if (considerDanger)
            {
                rightSideAngle += angles[i] * (interest[i] * (1 - danger[i]));
            }
            else
            {
                rightSideAngle += angles[i] * (interest[i] * (1 - danger[i]));
            }
        }
        return rightSideAngle;
    }

    float getAverageObstacleDistance(float contextLookAheadDistance)
    {
        float averageDistance = 0;
        for (int i = 0; i < raycastHits.Length; i++)
        {
            if (rayHits[i] == false)
            {
                averageDistance += contextLookAheadDistance / raycastHits.Length;
            }
            else
            {
                averageDistance += raycastHits[i].distance / raycastHits.Length;

            }
        }
        return averageDistance;
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

        //Debug.DrawLine(car.transform.position, point2, Color.red);
        //Debug.DrawLine(car.transform.position, point3, Color.red);

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

        if (totalRotation.y < -360 + rotationThresholdoffset)
        {
            totalRotation.y = 0;
            stuntsCount[1]++;
        }
        else if (totalRotation.y > 360 - rotationThresholdoffset)
        {
            totalRotation.y = 0;
            stuntsCount[1]++;

        }

        if (totalRotation.z < (-360 + offset + rotationThresholdoffset))
        {
            totalRotation.z = 0;
            stuntsCount[2]++;
        }
        else if (totalRotation.z > (360 - offset - rotationThresholdoffset))
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
                    //Debug.Log("Time to complete stunt: " + (Time.time - initialTime));
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
                    //Debug.Log("Time to complete stunt: " + (Time.time - initialTime));
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
                    //Debug.Log("Time to complete stunt: " + (Time.time - initialTime));
                    yield break;
                }
                yield return null;
            }
            if (!car.isCarMidAir())
            {
                myRunningCoroutine = null;
                for (int i = 0; i < car.wheels.Length; i++)
                {
                    float previousGrip = car.wheels[i].tireGripFactor;
                    car.wheels[i].tireGripFactor = 0;
                    StartCoroutine(car.wheels[i].disengageGrip(previousGrip));
                }
                yield break;
            }
            yield return null;

        }

    }

    void AINitroBehaviour()
    {

        if ((velocityDiff < -5 || float.IsNaN(velocityDiff)) && car.nitroSystem.nitroDelay == 0 && car.status == CarObj.CarStatus.RUNNING && !car.isCarMidAir())
        {
            if (car.nitroSystem.isOverBoosting)
            {
                if (car.nitroSystem.AIoverBoostDisengageValue > (car.nitroSystem.nitroOverBoostValue * nitroDeactivationThreshold))
                {
                    car.nitroSystem.nitroOn = true;
                    car.nitroSystem.nitroDelayInit = true;
                    car.activateNitro();
                }
                else
                {
                    car.nitroSystem.nitroOn = false;
                    car.nitroSystem.nitroValue = Mathf.Clamp(car.nitroSystem.nitroValue - Time.fixedDeltaTime * car.nitroSystem.nitroCoolRate, 0, 1);
                    if (car.nitroSystem.nitroDelayInit)
                    {
                        car.nitroSystem.nitroDelay = (car.nitroSystem.isOverBoosting) ? car.nitroSystem.nitroOverBoostDelayTime : car.nitroSystem.nitroDelayTime;
                        car.nitroSystem.nitroDelayInit = false;
                    }
                    car.nitroSystem.isOverBoosting = false;
                    car.engine.nitroTorque = 0;
                    StartCoroutine(car.NitroReactivation());
                }
            }
            else
            {
                car.nitroSystem.nitroOn = true;
                car.nitroSystem.nitroDelayInit = true;
                car.activateNitro();
            }

        }
        else
        {
            car.nitroSystem.nitroOn = false;
            car.nitroSystem.nitroValue = Mathf.Clamp(car.nitroSystem.nitroValue - Time.fixedDeltaTime * car.nitroSystem.nitroCoolRate, 0, 1);
            if (car.nitroSystem.nitroDelayInit)
            {
                car.nitroSystem.nitroDelay = (car.nitroSystem.isOverBoosting) ? car.nitroSystem.nitroOverBoostDelayTime : car.nitroSystem.nitroDelayTime;
                car.nitroSystem.nitroDelayInit = false;
            }
            car.nitroSystem.isOverBoosting = false;
            car.engine.nitroTorque = 0;
            StartCoroutine(car.nitroSystem.NitroReactivation());
        }

    }

    void engageStabilization() // Function for being able to stabilize the car when in mid air, and front the car from rolling after a jump. Also can flip an upside down car if near stationary
    {
        Vector3 currentForward = transform.forward;
        Vector3 closestPointSteerToLookAhead = lineToFollow[currentSpline].GetPoint(normalizedT); // Get actual point with lookahead distance

        Vector3 targetDirection = (closestPointSteerToLookAhead - transform.position).normalized; // Example: towards another object
        Vector3 error = Vector3.Cross(currentForward, targetDirection);
        float angleDifference = Vector3.Angle(currentForward, targetDirection);
        float torqueMagnitude = angleDifference * car.airRotationAmount; // Adjust rotationSpeedMultiplier as needed

        // Calculate P
        Vector3 P = proportionalGain * error; // The P value that will help lessen the error (Rotate to align the car to normal)
                                              // Calculate D term
        Vector3 errorRateOfChange = (error - errorLast) / Time.fixedDeltaTime; // Rate of which the error changes, used for D value
        errorLast = error;


        Vector3 D = derivativeGain * errorRateOfChange; // This value is responsible for preventing an overshoot of the target.


        // Calculate I term
        integrationStored = integrationStored + (error * Time.fixedDeltaTime); // This is responsible for trying to make the rotation of the car align with the target as much as possible
        Vector3 I = integrationStored * integralGain;


        car.rb.AddTorque(car.airRotationAmount * P + D + I, ForceMode.Impulse); // Combine and add all forces as torques

        Debug.DrawLine(car.transform.position, closestPointSteerToLookAhead, Color.cyan);

    }

    public void respawnVehicle()
    {
        transform.position = lineToFollow[currentSpline].FindNearestPointTo(car.transform.position, out float closestT, 95);
        transform.rotation = Quaternion.LookRotation(lineToFollow[currentSpline].GetTangent(closestT), Vector3.up);
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
