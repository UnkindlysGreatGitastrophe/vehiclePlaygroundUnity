using System;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;
using static CarObj;

public class UIScript : MonoBehaviour
{
    [Header("References")]
    public CarObj car;
    public EngineObj engine;
    public Image[] engineRPMPoints;

    [Header("EngineRPMGauge")]
    public float engineRPMFillAmount;
    public float engineRPMPerPoint;
    private Color tempColor;
    private bool emptyOut = false;

    [Header("GearShiftIndicator")]
    public TextMeshProUGUI gearText;
    public string[] shiftIndicatorText;
    [Header("Speedometer")]
    public TextMeshProUGUI speedometerText;

    [Header("Boost Indicator")]
    public Image nitroFillImage;
    public Sprite[] nitroIcons;
    public Image nitroIcon;
    public float maxFillAmount = 0.83f;
    public Transform boostTipPivot;

    public float flickerTime = 0.7f;
    public float maxFlickerTime = 0.7f;
    private bool overboostInit = true;

    [Header("Boost Indicator")]
    public Image healthFillImage;
    public float healthMaxFillUIOffset = 0.428f;
    public float healthMinFillUIOffset = 0.33f;



    [Header("Stunt UI")]
    public TextMeshProUGUI stuntDisplayText;
    public TextMeshProUGUI nitroGainDisplayText;
    public TextMeshProUGUI nitroPowerMultiplierText;
    public Animator stuntUIAnimation;


    // Start is called before the first frame update
    void Start()
    {
        engine = car.GetComponentInChildren<EngineObj>();
        engineRPMPerPoint = (float) 1 / engineRPMPoints.Length * (car.engine.rpmLimit-engine.overRevPenalty);
        shiftIndicatorText = new string[car.gearBox.numOfGears+1];
        shiftIndicatorText[0] = "R";
        shiftIndicatorText[1] = "N";
        for (int i = 2; i < shiftIndicatorText.Length; i++)
        {
            shiftIndicatorText[i] = (i - 1).ToString();
        }
    }

    // Update is called once per frame
    void Update()
    {
        engineRPMFillAmount = (engine.engineAngularVelocity * engine.AV_2_RPM - engine.rpmLimitIdle) / (engine.rpmLimit-engine.overRevPenalty) * engine.rpmLimit;
        updateRPMReader();
        updateGearReader();
        updateSpeedoMeter();
        updateBoostGauge();
        updateHealthGauge();
    }

    void updateRPMReader()
    {
        for (int i = 0; i < engineRPMPoints.Length; i++)
        {  
            tempColor = engineRPMPoints[i].color;
            if (!emptyOut)
                tempColor.a = Mathf.Clamp(engineRPMFillAmount/(engineRPMPerPoint*(i+1)),0,1);
            else
                tempColor.a = 0;
            engineRPMPoints[i].color = tempColor;
            if (tempColor.a != 1)
            {
                emptyOut = true;
            }
        }
        emptyOut = false;
    }

    void updateGearReader()
    {
        gearText.text = shiftIndicatorText[car.gearBox.currentGear].ToString();
    }

    void updateSpeedoMeter()
    {
        speedometerText.text = Mathf.Abs(Mathf.RoundToInt(car.carSpeed)).ToString("000");
    }

    void updateBoostGauge()
    {
        nitroFillImage.fillAmount = (car.nitroSystem.nitroValue - 0) / (1 - 0) * (maxFillAmount - 0) + 0;
        Color newColor = Color.Lerp(Color.yellow, Color.red, car.nitroSystem.nitroValue);
        nitroFillImage.color = newColor;
        boostTipPivot.localEulerAngles = new Vector3(0, 0,-nitroFillImage.fillAmount*360f);

        if (!car.nitroSystem.isOverBoosting)
        {
            overboostInit = true;
            flickerTime = 0f;
        }
        if (car.nitroSystem.nitroValue == 0)
        {
            nitroIcon.sprite = nitroIcons[0];
        }
        else
        {
            if (car.nitroSystem.nitroValue > 0 && !car.nitroSystem.isOverBoosting)
            {
                nitroIcon.sprite = nitroIcons[1];
            }
            if (car.nitroSystem.isOverBoosting && car.nitroSystem.nitroOn)
            {
                if (overboostInit == true)
                {
                    nitroIcon.sprite = nitroIcons[2];
                    flickerTime = maxFlickerTime;
                    overboostInit = false;
                }
                if (flickerTime < 0 && nitroIcon.sprite == nitroIcons[3])
                {
                    nitroIcon.sprite = nitroIcons[2];
                    flickerTime = maxFlickerTime;
                }
                if (flickerTime < 0 && nitroIcon.sprite == nitroIcons[2])
                {
                    nitroIcon.sprite = nitroIcons[3];
                    flickerTime = maxFlickerTime;
                }
                flickerTime = flickerTime - Time.deltaTime;

            }

        }
        
    }

    void updateHealthGauge()
    {
        float healthRatio = (car.currentCarHealth) / car.CarHealth;
        healthFillImage.fillAmount = Remap(healthRatio, 0, 1, healthMinFillUIOffset, healthMaxFillUIOffset);
        
    }

    public void showcaseStuntText(Dictionary<StuntType, int> recordedStunts, float boostMultiplier)
    {
        int count = recordedStunts.Count-1;
        int idx = 0;
        string suffix;
        string stuntText = "";
        foreach(KeyValuePair<StuntType, int> entry in recordedStunts)
            {
                if (idx == count)
                {
                    suffix = "!!!";
                }
                else
                {   
                    suffix = "+ ";
                }
                if (entry.Key == StuntType.SPIN360)
                {
                    stuntText = stuntText + (360 * entry.Value).ToString() + " " + suffix;
                }
                else if (entry.Key == StuntType.FRONTFLIP)
                {
                    stuntText = stuntText + entry.Value.ToString() + "X Frontflip " + suffix; 
                }
                else if (entry.Key == StuntType.BACKFLIP)
                {
                    stuntText = stuntText + entry.Value.ToString() + "X Backflip " + suffix; 
                }
                else if (entry.Key == StuntType.BARRELROLL)
                {
                    stuntText = stuntText + entry.Value.ToString() + "X Barrel Roll " + suffix; 
                }
                idx++;
                // do something with entry.Value or entry.Key
            }
        stuntDisplayText.text = stuntText;
        nitroGainDisplayText.text = "+" + boostMultiplier.ToString()+ "X " + "Nitro Power Bonus";
        nitroPowerMultiplierText.text = car.nitroSystem.boostStuntMultiplier.ToString("#.00") + "x";
        stuntDisplayText.enabled = true;
        nitroGainDisplayText.enabled = true;
        stuntUIAnimation.enabled = true;
        StartCoroutine(RemoveNitroUI());

    }

    private IEnumerator RemoveNitroUI()
    {
       
        yield return new WaitForSeconds(3);
        stuntDisplayText.enabled = false;
        nitroGainDisplayText.enabled = false;
        stuntUIAnimation.enabled = false;

        
    }

    public static float Remap (float input, float rangeMin, float rangeEnd, float newRangeMin,  float newRangeEnd)
    {
        float t = Mathf.InverseLerp(rangeMin, rangeEnd, input);
        float output = Mathf.Lerp(newRangeMin, newRangeEnd, t);
        return output;
    }
}
