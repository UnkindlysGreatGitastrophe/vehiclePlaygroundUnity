using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class StuntObj : MonoBehaviour
{
    
    [Header("Reference")] 
    public CarObj car;

    [Header("Stunts")]
    [Tooltip("Rate of which the car can rotate in the air")]
    public float airRotationAmount = 50f; // Rate of which the rotation of the car is performed when in the air
    [Tooltip("Stunts!")]
    public int numFrontFlips = 0; // Stunt Count
    [Tooltip("Stunts!")]
    public int numBackFlips = 0;
    [Tooltip("Stunts!")]
    public int numLeftBarrelRolls = 0;
    [Tooltip("Stunts!")]
    public int numRightBarrelRolls = 0;
    [Tooltip("Stunts!")]
    public int numLeft360s = 0;
    [Tooltip("Stunts!")]
    public int numRight360s = 0;
    [Tooltip("Total amount of rotation made in a particular axis relative to an original rotation")]
    public float lastStuntScore = 0;

    public Vector3 totalRotation = Vector3.zero; // Total rotation performed in the car relative to a lastRotation
    [Tooltip("The pivot point of which total rotation is calclated off of")]
    public Quaternion lastRotation; // The pivot point of which total rotation is calclated off of
    public List<CarObj.StuntType> stuntsInStreak;
    public List<List<CarObj.StuntType>> allRecordedStunts = new List<List<CarObj.StuntType>>();

    public Vector3 lastRotationPID; // The pivot point of which total rotation is calclated off of

    [Tooltip("The Euler representation of the difference between the current and last rotation")]
    private Vector3 angleDiffs; // The difference between the current and lastRotation
    [Tooltip("When true, the last rotation prior to taking a jump will be stored for tracking stunts")]
    bool originInit = false; // Bool that initializes the lastRotation the moment we catch air
    // Start is called before the first frame update
    void AerialRotation() // Script for allowing mid-air control using the same inputs for on ground driving
    {


        // Allow the car to Rotate
        float h = Input.GetAxis("Horizontal") * airRotationAmount * Time.deltaTime;
        float v = Input.GetAxis("Vertical") * airRotationAmount * Time.deltaTime;
        if (Input.GetKey(KeyCode.Space))
        {
            if (Input.GetKey(KeyCode.Space))
            car.rb.AddTorque(car.rb.transform.right * v, ForceMode.VelocityChange);

            if (car.allowBarrelRoll) // If barrel rolls are allowed, flat spins are replaced with barrel rolls instead
            {
                car.rb.AddRelativeTorque(0f, 0f, h, ForceMode.VelocityChange);
            }
        }
        
        else
        {
            car.rb.AddTorque(car.rb.transform.up * h, ForceMode.VelocityChange);
        }
        //Debug.DrawRay(rb.position, new Vector3(rb.velocity.x,Physics.gravity.y, rb.velocity.z), Color.cyan);
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


        // Increase stunt count for when a full revolution is complete.

        if (totalRotation.x < -270f) 
        {
            totalRotation.x = 0;
            numFrontFlips++;
            stuntsInStreak.Add(CarObj.StuntType.FRONTFLIP);
        }
        else if (totalRotation.x > 270f)
        {
            totalRotation.x = 0;
            numBackFlips++;
            stuntsInStreak.Add(CarObj.StuntType.BACKFLIP);

        }

        totalRotation.y += angleDiffs.y;

        if (totalRotation.y < -270f)
        {
            totalRotation.y = 0;
            numRight360s++;
            stuntsInStreak.Add(CarObj.StuntType.SPIN360);

        }
        else if (totalRotation.y > 270f)
        {
            totalRotation.y = 0;
            numLeft360s++;
            stuntsInStreak.Add(CarObj.StuntType.SPIN360);

        }

        totalRotation.z += angleDiffs.z;

        if (totalRotation.z < -270f)
        {
            totalRotation.z = 0;
            numRightBarrelRolls++;
            stuntsInStreak.Add(CarObj.StuntType.BARRELROLL);

        }
        else if (totalRotation.z > 270f)
        {
            totalRotation.z = 0;
            numLeftBarrelRolls++;
            stuntsInStreak.Add(CarObj.StuntType.BARRELROLL);

        }
    }

    float calculateStuntScore()
    {
        if (stuntsInStreak.Count == 0)
        {
            return 0f;
        }

        float totalScore = 0f;
        Dictionary<CarObj.StuntType, int> recordedStunts = new Dictionary<CarObj.StuntType,int>();
        for (int i = 0; i < stuntsInStreak.Count; i++)
        {
            if (!recordedStunts.ContainsKey((CarObj.StuntType)stuntsInStreak[i]))
            {
                recordedStunts.Add((CarObj.StuntType)stuntsInStreak[i], 1);
            }
            else
            {
                recordedStunts[stuntsInStreak[i]] += 1;
            }
        }

        foreach(KeyValuePair<CarObj.StuntType, int> entry in recordedStunts)
            {
                float baseStuntScore = 0;
                for (int i = 0; i < entry.Value; i++)
                {
                    baseStuntScore += car.stuntScore[entry.Key] * car.repeatStuntPenalty[Mathf.Min(i,2)];
                }
                totalScore += baseStuntScore;
                // do something with entry.Value or entry.Key
            }
        if (car.ui != null)
        {
            if (!car.stuntProcessing)
            {
                StartCoroutine(processStunt(recordedStunts, totalScore));
                car.stuntProcessing = true;
            }
            //ui.showcaseStuntText(recordedStunts, totalScore);
            
        }
        
        return totalScore;
    }

    void PID() // Function for being able to stabilize the car when in mid air, and front the car from rolling after a jump. Also can flip an upside down car if near stationary
    {
        
        if (Mathf.Abs(car.rb.transform.localRotation.eulerAngles.z) > 60f && Mathf.Abs(car.carSpeed) <= 5f) { // If the car is upside down and nearly going at 0 km/h, then we allow the player to flip the car.
        car.rb.AddRelativeTorque(0f, 0f, 30, ForceMode.Acceleration);
        }

        bool isHit = Physics.Raycast(car.rb.position, new Vector3(car.rb.velocity.x,Physics.gravity.y, car.rb.velocity.z), out car.RaycastDir); // Raycast, predicts where the car will land approximately.

        // Debug.DrawRay(rb.position, new Vector3(rb.velocity.x,Physics.gravity.y, rb.velocity.z), Color.cyan);


        if (isHit) // If there is a ground underneath the car...
        {

            Debug.DrawRay(car.RaycastDir.point, car.RaycastDir.normal, Color.magenta);
            Debug.DrawRay(car.rb.position, car.rb.transform.up, Color.magenta);


            

            Vector3 targetValueUP = car.RaycastDir.normal; // Get the normal vector of the ground, this will be our target value we want the car's rotation to match.
            //Vector3 error = targetValue - rb.transform.up;
            Vector3 error = Vector3.Cross(car.rb.transform.up, targetValueUP); // The discrepency between the target and current rotation of the car.
            // Calculate P
            Vector3 P = car.proportionalGain * error; // The P value that will help lessen the error (Rotate to align the car to normal)
            // Calculate D term
            Vector3 errorRateOfChange = (error - car.errorLast) / Time.fixedDeltaTime; // Rate of which the error changes, used for D value
            car.errorLast = error;


            Vector3 D = car.derivativeGain * errorRateOfChange; // This value is responsible for preventing an overshoot of the target.
            

                        // Calculate I term
            car.integrationStored = car.integrationStored + (error * Time.fixedDeltaTime); // This is responsible for trying to make the rotation of the car align with the target as much as possible
            Vector3 I =  car.integrationStored * car.integralGain;


            car.rb.AddTorque(car.PIDstrength * P+D+I, ForceMode.Impulse); // Combine and add all forces as torques
        }


        // float error = targetValue - currentValue;

    }

    private IEnumerator processStunt(Dictionary<CarObj.StuntType, int> recordedStunts, float totalScore)
    {
        float time = 0f;
        while (true)
        {
            yield return .1;
            if (!car.isCarMidAir())
            {
                time += Time.deltaTime;
                if (time >= 1)
                {
                    car.nitroSystem.boostStuntMultiplier += totalScore;
                    car.clutch.clutchCapacity = car.nitroSystem.boostStuntMultiplier;
                    car.clutch.clutchMaxTorq = car.clutch.clutchCapacity * car.clutch.clutchStiffness;
                    car.ui.showcaseStuntText(recordedStunts, totalScore);
                    car.stuntProcessing = false;
                    allRecordedStunts.Add(stuntsInStreak);
                    stuntsInStreak.Clear();
                    yield break;
                }
                    
            }
            else
            {
                time = 0;
            }
        }
        
        
    }
}
