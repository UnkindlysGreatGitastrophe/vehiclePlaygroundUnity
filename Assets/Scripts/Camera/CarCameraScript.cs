using System.Collections;
using Cinemachine;
using UnityEngine;

public class CarCameraScript : MonoBehaviour
{
    public CinemachineVirtualCamera vcam;
    public float minAmpGain = 0, maxAmpGain = 0.2f;
    public float minFreqGain = 0.01f, maxFreqGain = 0.07f;
    public float minSpeedAmp = 75f, maxSpeedAmp = 300;
    public float minSpeedFreq = 50f, maxSpeedFreq = 300;
    public CinemachineTransposer.BindingMode groundcam = CinemachineTransposer.BindingMode.LockToTargetWithWorldUp;

    public CinemachineTransposer.BindingMode midaircam = CinemachineTransposer.BindingMode.SimpleFollowWithWorldUp;
    public CarObj car;


    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void FixedUpdate()
    {
        cameraBehaviour();

    }

    private void cameraBehaviour()
    {
        var sameAsFollowTarget = vcam.GetCinemachineComponent<CinemachineTransposer>();
        var noise = vcam.GetCinemachineComponent<CinemachineBasicMultiChannelPerlin>();
        noise.m_AmplitudeGain = Mathf.Clamp((car.carSpeed - minSpeedAmp) / maxSpeedAmp * maxAmpGain, minAmpGain, maxAmpGain);
        noise.m_FrequencyGain = Mathf.Clamp((car.carSpeed - minSpeedFreq) / maxSpeedFreq * maxFreqGain, minFreqGain, maxFreqGain);

        if (car.isCarPartiallyMidAir())
        {
            
            StartCoroutine(TransitionToAirCam(sameAsFollowTarget));
        }
        else
        {
            sameAsFollowTarget.m_BindingMode = groundcam;
        }

        if (Input.GetKey(KeyCode.V))
        {
            sameAsFollowTarget.m_FollowOffset.z = 6f;

        }
        else
        {
            sameAsFollowTarget.m_FollowOffset.z = -6f;


        }

    }

    private IEnumerator TransitionToAirCam(CinemachineTransposer sameAsFollowTarget)
    {
        yield return new WaitForSeconds(0.5f);
        if (car.isCarMidAir())
            sameAsFollowTarget.m_BindingMode = midaircam;
    }
}
