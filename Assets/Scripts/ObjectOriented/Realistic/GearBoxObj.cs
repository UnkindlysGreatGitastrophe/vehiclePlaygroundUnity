using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GearBoxObj : MonoBehaviour
{

    [Header("GearBox Params")]

    public int numOfGears;
    public float[] gearRatios;
    public float shiftTime = 1;
    public float finalDriveGear = 3.6f;


    [Header("GearBox Outputs")]
    [SerializeField] private int currentGear = 1;
    [SerializeField] private bool gearEngaged = true;
    

    // Update is called once per frame
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

    public float get_ratio()
    {
        return gearRatios[currentGear];
    }
    #endregion
}
