using System.Collections;
using Cinemachine;
using UnityEngine;

public class CarCameraScript : MonoBehaviour
{
    public CinemachineVirtualCamera groundVCam; public CinemachineVirtualCamera airVCam;
    private CinemachineBrain cinemachineBrain;
    public float minAmpGain = 0, maxAmpGain = 0.2f;
    public float minFreqGain = 0.01f, maxFreqGain = 0.07f;
    public float minSpeedAmp = 75f, maxSpeedAmp = 300;
    public float minSpeedFreq = 50f, maxSpeedFreq = 300;
    public CinemachineTransposer.BindingMode groundcam = CinemachineTransposer.BindingMode.LockToTargetWithWorldUp;

    public CinemachineTransposer.BindingMode midaircam = CinemachineTransposer.BindingMode.SimpleFollowWithWorldUp;
    public CarObj car;
    CinemachineTransposer sameAsFollowTarget;
    CinemachineBasicMultiChannelPerlin noise; // INEFFICIENT???



    // Start is called before the first frame update
    void Start()
    {
        cinemachineBrain = this.GetComponent<CinemachineBrain>();
        Transform CarBodyFolder = car.transform.Find("DeformableObjects");
        sameAsFollowTarget = groundVCam.GetCinemachineComponent<CinemachineTransposer>(); // INEFFICIENT???
        noise = groundVCam.GetCinemachineComponent<CinemachineBasicMultiChannelPerlin>(); // INEFFICIENT???
        foreach (Transform child in CarBodyFolder)
        {
            if (child.name.Contains("Body"))
            {
                groundVCam.m_Follow = child;
                groundVCam.m_LookAt = child;
                airVCam.m_Follow = child;
                airVCam.m_LookAt = child;
            }
        }
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        cameraBehaviour();

    }

    private void cameraBehaviour()
    {
        noise.m_AmplitudeGain = Mathf.Clamp((car.carSpeed - minSpeedAmp) / maxSpeedAmp * maxAmpGain, minAmpGain, maxAmpGain);
        noise.m_FrequencyGain = Mathf.Clamp((car.carSpeed - minSpeedFreq) / maxSpeedFreq * maxFreqGain, minFreqGain, maxFreqGain);

        if (car.isCarMidAir())
        {
            groundVCam.enabled = false;
            airVCam.enabled = true;
            cinemachineBrain.m_DefaultBlend.m_Time = 0.1f;

        }
        else
        {
            groundVCam.enabled = true;
            airVCam.enabled = false;
            sameAsFollowTarget.m_BindingMode = groundcam;
            cinemachineBrain.m_DefaultBlend.m_Time = 1f;
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
    
    void CopyCameraProperties(CinemachineVirtualCamera source, CinemachineVirtualCamera destination)
    {
        // Copy component values
        destination.Priority = source.Priority;
        destination.m_Lens = source.m_Lens;
        destination.m_LookAt = source.m_LookAt;
        destination.m_Follow = source.m_Follow;
        destination.m_StandbyUpdate = source.m_StandbyUpdate;

        // Copy component settings
        destination.m_Lens.NearClipPlane = source.m_Lens.NearClipPlane;
        destination.m_Lens.FarClipPlane = source.m_Lens.FarClipPlane;
        destination.m_Lens.FieldOfView = source.m_Lens.FieldOfView;
        destination.m_Lens.OrthographicSize = source.m_Lens.OrthographicSize;
        destination.m_Lens.Orthographic = source.m_Lens.Orthographic;

        // Copy other settings like target offset, etc. if needed
    }
}
