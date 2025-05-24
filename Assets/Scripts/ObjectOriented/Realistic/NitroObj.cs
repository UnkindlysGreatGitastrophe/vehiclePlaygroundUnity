using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class NitroObj : MonoBehaviour
{
    [Header("References")]
    public CarObj car;

    [Header("Nitro Properties")]

    [Tooltip("The time it takes to be able to reactivate nitro once the player lays off the boost")]
    public float nitroDelay = 0f; // The time it takes to be able to reactivate nitro once the player lays off the boost
    public float nitroDelayTime = 0.4f; // In seconds, how much the player will need to wait until they are able to use nitro again
    public float nitroOverBoostDelayTime = 1.5f; // In seconds, how much the player will need to wait until they are able to use nitro again after overboosting
    public float nitroHeatRate = 1.5f; // The rate of which the heating of the nitro increases as the nitro is used
    public float nitroCoolRate = 1; // The rate of which the heating of the nitro decreases as the nitro is disengaged
    public float airNitroCoolRate = 2; // The rate of which the heating of the nitro decreases as the nitro is disengaged while in mid-air
    public float maxOverBoostPenalty = 3; // In seconds, this determines the amount of time a car can spend overboosting before it explodes
    public float nitroValue = 0; // From a range of 0 (No nitro use) to 1 (Overheating), this variable determines how hot the nitro temperature is
    internal float nitroOverBoostValue = 0; // From a range of 0 (Not overboosting) to the maxOverBoostPenalty (Car explodes from overboosting), this variable determines how much overboosting a player is doing
    public bool isOverBoosting; // Determines if the car is in the overboost zone
    internal bool nitroDelayInit; // Used to initialize delays after the player stops using nitro
    public bool nitroOn; // Indicates if the nitro is being used or not
    public float boostStuntMultiplier = 1;

    public float kineticBoost = 1500f;
    public float kineticBoostMultiplier = 3.5f;
    public float nitrousPower = 0.5f;

    // Start is called before the first frame update
    void Start()
    {
        if (transform.parent.GetComponent<CarObj>() != null)
        {
            car = transform.parent.GetComponent<CarObj>();
        }
        if (transform.parent.parent.GetComponent<CarObj>() != null)
        {
            car = transform.parent.parent.GetComponent<CarObj>();
        }
    }

    // Update is called once per frame
    void Update()
    {
        
    }
    #region Nitro

    public void activateNitro()
    {
        if (nitroOn)
        {     
            applyNitro();
            nitroDelayInit = true;            
            nitroValue = Mathf.Clamp(nitroValue + Time.deltaTime * nitroHeatRate, 0, 1);
            if (nitroValue == 1 && !isOverBoosting)
            {
                Debug.Log("OverBoost Alert!");
                isOverBoosting = true;
                StartCoroutine(overBoostCountDown());
            }
        }
        
    }

    public void applyNitro()
    {
        car.rb.AddForce(transform.forward * kineticBoost * (kineticBoostMultiplier * (car.isCarMidAir() ? 1 : 0)) * (Mathf.Max(1,boostStuntMultiplier/2)));
        car.engine.nitroTorque = car.engine.initialTorque * nitrousPower * boostStuntMultiplier;
    }

    private IEnumerator overBoostCountDown()
    {
        while (nitroOn)
        {
            nitroOverBoostValue += 1;
            if (nitroOverBoostValue >= maxOverBoostPenalty)
            {
                Debug.Log("Kaboom!");
            } 
            yield return new WaitForSeconds(1);

        }
        
    }

     public IEnumerator NitroReactivation()
    {
        while (nitroOn)
        {
            yield return new WaitForSeconds(1);
            
            Debug.Log("Still Boosting...");
            
        }
        nitroOverBoostValue = 0;
        nitroDelay = Mathf.Clamp(nitroDelay - 1 * Time.deltaTime , 0, 1);
        
    }
    #endregion
}
