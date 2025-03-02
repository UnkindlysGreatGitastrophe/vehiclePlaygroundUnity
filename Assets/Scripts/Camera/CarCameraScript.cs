using Cinemachine;
using UnityEngine;

public class CarCameraScript : MonoBehaviour
{
    public CinemachineVirtualCamera vcam;
    public float minAmpGain = 0, maxAmpGain = 0.2f;
    public float minFreqGain = 0.01f, maxFreqGain = 0.07f;
    public float minSpeedAmp = 75f, maxSpeedAmp = 300;
    public float minSpeedFreq = 50f, maxSpeedFreq = 300;
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
        noise.m_AmplitudeGain = Mathf.Clamp((car.carSpeed - minSpeedAmp) / maxSpeedAmp * maxAmpGain, minAmpGain,maxAmpGain );
        noise.m_FrequencyGain = Mathf.Clamp((car.carSpeed - minSpeedFreq) / maxSpeedFreq * maxFreqGain, minFreqGain,maxFreqGain );

        if (car.isCarMidAir())
        {
            sameAsFollowTarget.m_BindingMode = CinemachineTransposer.BindingMode.SimpleFollowWithWorldUp;
        }
        else
        {
            sameAsFollowTarget.m_BindingMode = CinemachineTransposer.BindingMode.LockToTargetWithWorldUp;
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
   
}
