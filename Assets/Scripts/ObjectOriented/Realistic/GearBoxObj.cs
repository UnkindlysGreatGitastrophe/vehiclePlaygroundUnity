using System;
using System.Collections;
using System.Collections.Generic;
using Baracuda.Monitoring;
using UnityEngine;
using UnityEngine.Assertions;

public class GearBoxObj : MonoBehaviour
{
    public enum GearboxType {MANUAL, AUTOMATIC};
    [Header("References")]
    public CarObj car;

    [Header("GearBox Params")]

    public int numOfGears;
    public float[] gearRatios;
    public float shiftTime = 1;
    public float finalDriveGear = 3.6f;
    public GearboxType Transmissiontype;



    [Header("GearBox Outputs")]
    [Monitor] [SerializeField] private int currentGear = 1;
    [Monitor] [SerializeField] public bool gearEngaged = true;
    

void Start(){
            this.StartMonitoring();
            numOfGears = gearRatios.Length;
            car = transform.GetComponentInParent<CarObj>();

}
    // Update is called once per frame
    void Update()
    {
        if (Transmissiontype == GearboxType.AUTOMATIC)
        {
            AutoGearBoxManager();
        }
        if (Input.GetKeyDown(KeyCode.Q))
        {
            GearUP();
        }
        if (Input.GetKeyDown(KeyCode.E))
        {
            GearDOWN();
        }
    }


    #region Automatic
    private void AutoGearBoxManager()
    {
        if (car.carSpeed >= GetMaxGearSpeed(currentGear))
        {
            GearUP();
        }
        else if (car.carSpeed < 0.85 *GetMaxGearSpeed(currentGear - 1)  && currentGear > 2)
        {
            GearDOWN();
        }
    }

    #endregion

    #region GearBox
    private void GearUP()
    {
        if (currentGear < numOfGears-1 && gearEngaged)
        {
            gearEngaged = false;
            //Debug.Log(GetMaxGearSpeed(currentGear + 1));
            StartCoroutine(UpShiftTime(currentGear + 1));
        }
    }

    private float GetMaxGearSpeed(int gear)
    {
        /*
            EXAMPLE:
                    let's say you have a tyre of a radius of 0.45
                    the circumference is more or less 2.8 meters - that means the tyre is traveling 2.8 meters per revolution
                    max engine revs = 6800
                    revolutions per minute
                    divide that by 60 and you get around 113 revs per second
                    then taking into account gear ratios the wheel rotates more or less 7.8 times a second
                    2.8 meters per revolution, 7.8 revolutions per second - 22.3 meters per second
                    multiply by 3.6 - 80 KMH
                    that's the correct calculation
        */
        if (car.gearBox.get_specific_ratio(gear) == 0)
        {
            return 0;
        }
        float maxRPS = (car.engine.rpmLimit - (car.engine.overRevPenalty/2)) / 60; // Maximum Revs per second
        float wheelRPS = maxRPS / car.gearBox.get_specific_ratio(gear) / car.gearBox.finalDriveGear; // Wheel Revs Per Second (Rad/s)
        float tireCircumference = car.wheels[0].tireRadius * 2 * Mathf.PI; // Assuming all tires are the same radius (Not good for dragsters)
        float metersPS = tireCircumference * wheelRPS;
        float GetMaxGearSpeed = metersPS * 3.6f; // UNIT IS KM/H

        return GetMaxGearSpeed;
    }

    private IEnumerator UpShiftTime(int shiftDirection)
    {
        currentGear = 1;
        car.Throttle = 0;
        yield return new WaitForSeconds(shiftTime);
        currentGear = shiftDirection;
        gearEngaged = true;

    }

        private IEnumerator ShiftRevMatch(int shiftDirection)
    {
        int previousGear = currentGear;
        currentGear = 1;
        car.Throttle = 0;
        
        if (previousGear != 1 && car.carSpeed < GetMaxGearSpeed(shiftDirection))
        {
            Debug.Log("Rev Matching");
            float RPMDifference = car.engine.engineAngularVelocity * car.engine.AV_2_RPM - Mathf.Abs(car.GetAVGWheelRPM() * car.gearBox.get_specific_ratio(shiftDirection) * car.gearBox.finalDriveGear);
            while ( Mathf.Abs(RPMDifference) > 800)
            {
                if (RPMDifference > 0) // If engine is quicker than wheel RPM, lower throttle
                {
                    car.Throttle = Mathf.Clamp(car.Throttle + Time.fixedDeltaTime * -5, 0, 1);
                }
                else
                {
                    car.Throttle = Mathf.Clamp(car.Throttle + Time.fixedDeltaTime * 3, 0, 1);
                }
                RPMDifference = car.engine.engineAngularVelocity * car.engine.AV_2_RPM - (car.GetAVGWheelRPM() * car.gearBox.get_specific_ratio(shiftDirection) * car.gearBox.finalDriveGear);
                yield return null;
            }
            currentGear = shiftDirection;  
            gearEngaged = true;
        }
        else
        {
            currentGear = 1;
            car.Throttle = 0;
            yield return new WaitForSeconds(shiftTime);
            currentGear = shiftDirection;
            gearEngaged = true;
        }

    }


    private void GearDOWN()
    {
        if (currentGear > 0 && gearEngaged)
        {
        gearEngaged = false;
        //Debug.Log(GetMaxGearSpeed(currentGear - 1));
        StartCoroutine(ShiftRevMatch(currentGear - 1));
        }
    }

    public float get_ratio()
    {
        return gearRatios[currentGear];
    }

        public float get_specific_ratio(int gear)
    {
        Assert.IsTrue(gear > 0 && gear < numOfGears);
        return gearRatios[gear];
    }
    #endregion
}
