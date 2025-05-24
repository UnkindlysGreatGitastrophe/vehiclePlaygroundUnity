using Baracuda.Monitoring;
using UnityEngine;

public class ForcedInductionObj : MonoBehaviour
{
    public enum ForcedInductionType {TURBO,SUPERCHARGE};
    [Header("References")]
    public CarObj car;

    [Header("General Parameters")]
    public float boostPSI = 6;
    public ForcedInductionType type;

    [Header("TurboCharger Parameters")]
    public float inertia = 0.1f;
    public float valveInertia = 0.05f;

    [Header("SuperCharger Parameters")]
    public float psiPerRPM;
    // Start is called before the first frame update

    [Header("TurboCharger Output")]
    public float pressure;
    public float turboTorque;

    void Start()
    {
        this.StartMonitoring();
        if (transform.parent.GetComponent<CarObj>() != null)
        {
            car = transform.parent.GetComponent<CarObj>();
        }
        if (transform.parent.parent.GetComponent<CarObj>() != null)
        {
            car = transform.parent.parent.GetComponent<CarObj>();
        }
        psiPerRPM = boostPSI/car.engine.rpmLimit;
    }

    // Update is called once per frame
    public void calculateTurboPSI()
    {
        pressure += car.Throttle * (car.engine.engineAngularVelocity*car.engine.AV_2_RPM/car.engine.rpmLimit) * (1f - (pressure / boostPSI)) * Time.fixedDeltaTime / inertia;

        if (pressure > boostPSI * car.Throttle)
        {
            pressure -= (pressure / boostPSI) * Time.fixedDeltaTime / valveInertia;
        }
    }

    public void calculateSuperChargePSI()
    {
        pressure = car.engine.engineAngularVelocity*car.engine.AV_2_RPM * psiPerRPM  * car.Throttle;

        pressure = Mathf.Clamp(pressure,0, boostPSI);
    }


    public float convertPSItoTorque()
    {

        turboTorque = (car.Throttle*pressure+14.7f)/14.7f;
        return turboTorque;
    }
}
