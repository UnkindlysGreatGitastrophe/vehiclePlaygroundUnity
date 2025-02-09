using Cinemachine;
using UnityEngine;

public class CarCameraScript : MonoBehaviour
{
    public CinemachineVirtualCamera vcam;
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
        var framingTransposer = vcam.GetCinemachineComponent<CinemachineComposer>();
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
             framingTransposer.m_LookaheadTime = 0;
        }
        else
        {
            sameAsFollowTarget.m_FollowOffset.z = -6f;
            framingTransposer.m_LookaheadTime = 0.3f;

        }

   }
   
}
