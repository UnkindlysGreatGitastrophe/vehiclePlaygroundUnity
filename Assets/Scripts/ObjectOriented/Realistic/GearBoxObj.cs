using System;
using System.Collections;
using System.Collections.Generic;
using Baracuda.Monitoring;
using UnityEngine;

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
        if (car.engine.engineAngularVelocity * car.engine.AV_2_RPM >= car.engine.rpmLimit)
        {
            GearUP();
        }
        else if (car.engine.engineAngularVelocity * car.engine.AV_2_RPM <= car.engine.rpmLimitIdle && currentGear > 2)
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
            StartCoroutine(UpShiftTime(currentGear + 1));
        }
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
        currentGear = shiftDirection;  
        
        if (currentGear != 1 && previousGear != 1)
        {
            float RPMDifference = car.engine.engineAngularVelocity * car.engine.AV_2_RPM - Mathf.Abs(car.GetAVGWheelRPM() * car.gearBox.get_ratio() * car.gearBox.finalDriveGear);
            while ( Mathf.Abs(RPMDifference) > 800)
            {
                if (RPMDifference > 0) // If engine is quicker than wheel RPM, lower throttle
                {
                    car.Throttle = 0;
                }
                else
                {
                    car.Throttle = 1;
                }
                RPMDifference = car.engine.engineAngularVelocity * car.engine.AV_2_RPM - (car.GetAVGWheelRPM() * car.gearBox.get_ratio() * car.gearBox.finalDriveGear);
                yield return null;
            }
        }
        
        gearEngaged = true;

    }


    private void GearDOWN()
    {
        if (currentGear > 0 && gearEngaged)
        {
        gearEngaged = false;
        StartCoroutine(ShiftRevMatch(currentGear - 1));
        }
    }

    public float get_ratio()
    {
        return gearRatios[currentGear];
    }
    #endregion
}
