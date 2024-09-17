using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Net.Sockets;
using UnityEngine;

public class ArcadeCarObj : MonoBehaviour
{
    [Header("References")]
    public EngineObj engine;
    public GearBoxObj gearBox;
    public DriveTrainObj driveTrain;
    public WheelObj[] wheels;
    public WheelObj[] poweredWheels;

    [Header("Input")]
    public float Throttle;
    public bool ThrottleLock;
    public float Brake;
    public float steeringInput;

    [Header("Output")]
    public float Tc = 0; // Torque produced from the clutch after supplying it with the engine Torque
    public float torqueToWheel = 0; // Torque produced from the Gearbox after supplying it with the Torque CLutch
    // Start is called before the first frame update

    // Car Operation begins here:

}
    