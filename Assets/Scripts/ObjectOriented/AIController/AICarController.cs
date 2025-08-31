using BezierSolution;
using UnityEngine;
using Baracuda.Monitoring;
using UnityEngine.Rendering;
using UnityEngine.SocialPlatforms;
using System.Linq;
using System.Collections.Generic;

public class AICarController : MonoBehaviour
{

    [Header("References")]
    public CarObj car;
    public BezierSpline[] lineToFollow;

    [Monitor] float normalizedT;
    public float currentOffset = 10;
    [Monitor] private float maxCornerSpeed;
    [Monitor] private float maxBrakeDistance;
    [Monitor] private float lookAheadDistance;
    public float minLookAheadDist = 5;
    public float maxLookAheadDist = 50;
    [Monitor] public int currentSpline;
    public bool findNearestSpline;

    public float rangeThreshold = 50;
    public List<BezierSpline> foundBezierSplinePaths;

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


    void NormalDriving2()
    {

        // Cornering Speed Section

        maxBrakeDistance = calculateBrakeDistance(); // Get Maximum Brake Distance to the corner speed

        lookAheadDistance = Mathf.Clamp(maxBrakeDistance - (car.BrakeInput * 5 * Time.deltaTime), minLookAheadDist, maxLookAheadDist);

        Vector3 closestPointToLookAhead = lineToFollow[currentSpline].FindNearestPointTo(car.transform.position + (car.transform.forward * lookAheadDistance), out normalizedT, 95);

        Debug.DrawLine(car.transform.position, closestPointToLookAhead, Color.yellow);

        maxCornerSpeed = TestCornerSpeed2(normalizedT);

        print("Max Corner Speed: " + maxCornerSpeed + " km/h");


        // Steering Section

        float steeringLookAheadDistance = ClutchObj.Remap(car.carSpeed, 5, 200, 2, 50);

        Vector3 closestPointSteerToLookAhead = lineToFollow[currentSpline].FindNearestPointTo(car.transform.position + (car.transform.forward * steeringLookAheadDistance), 95);

        Debug.DrawLine(car.transform.position, closestPointSteerToLookAhead, Color.black);


        Vector3 worldDirection = closestPointSteerToLookAhead - car.transform.position;

        Vector3 localDirection = transform.InverseTransformDirection(worldDirection);

        float angleXZ = Mathf.Atan2(localDirection.x, localDirection.z) * Mathf.Rad2Deg;

        print("Steering Angle: " + angleXZ + " Degrees");

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
            currentSpline = System.Array.IndexOf(lineToFollow, foundBezierSplinePaths[Random.Range(0, foundBezierSplinePaths.Count)]);
            foundBezierSplinePaths.Clear();
            //findNearestSpline = false;
        }        
    }



}
