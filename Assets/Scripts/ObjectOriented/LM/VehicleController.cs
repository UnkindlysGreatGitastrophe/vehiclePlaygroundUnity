using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;
using UnityEditor;

namespace UDrive
{
	public class VehicleController : MonoBehaviour
	{

		[System.Serializable]
		public class Settings
		{
			//public Suspension.Settings suspensionSettings;
			//public InertiaBox.Settings inertiaBoxSettings;
			//public Drivetrain.Settings drivetrainSettings;
			//public WheelCollider.Settings frontWheelSettings,
				//rearWheelSettings;
			//public Wing.Settings frontWingSettings,
				//rearWingSettings;
		}

		public class State
		{
			public Vector3 velocityLS = Vector3.zero;
			public Vector3 accelerationLS = Vector3.zero;
			public float engineAngle = 0f;
			public float drivetrainAngle = 0f;
		}

		public Settings settings;
		public State state;
		//public VehicleInputData vehicleInputData;
		//[HideInInspector]
		//public InertiaBox inertiaBox;
		//[HideInInspector]
		//public Suspension suspension;
		//[HideInInspector]
		//public Drivetrain drivetrain;
		public Rigidbody vehicleBody;
		//public Wing frontWing,
			//rearWing;
		public WheelCollider[] wheelColliders;
		public GameObject[] wheelObjects;
		public GameObject[] wheelOrientationObjects;

		Vector3 lastFrameVelocityWS;

		void Awake()
		{
			state = new();
			Init();
		}

		void Init()
		{
			Setup();
		}

		void Setup()
		{
			//Initialize inertia box settings
			//inertiaBox = new InertiaBox(settings.inertiaBoxSettings);

			//Set up vehicle RigidBody
			vehicleBody = transform.gameObject.AddComponent<Rigidbody>();
			vehicleBody.interpolation = RigidbodyInterpolation.Interpolate;
			//vehicleBody.mass = inertiaBox.settings.mass;
			//vehicleBody.centerOfMass = inertiaBox.settings.offset + InertiaBox.CalculateCenterOfMass(inertiaBox.settings);
			//vehicleBody.inertiaTensor = inertiaBox.CalculateInertiaTensor();
			vehicleBody.inertiaTensorRotation = Quaternion.identity;

			//frontWing = new Wing(settings.frontWingSettings, vehicleBody);
			//rearWing = new Wing(settings.rearWingSettings, vehicleBody);

			//Add wheel GameObjects
			string[] names = { "FL", "FR", "RL", "RR" };
			wheelObjects = new GameObject[4];
			wheelOrientationObjects = new GameObject[4];
			wheelColliders = new WheelCollider[4];

			for (int i = 0; i < 4; i++)
			{
				GameObject wheelObject = new GameObject(names[i]);
				wheelObject.transform.parent = gameObject.transform;
				wheelObject.transform.rotation = gameObject.transform.rotation;
				GameObject wheelOrientationObject = new GameObject(names[i] + "O");
				wheelOrientationObject.transform.parent = wheelObject.transform;
				wheelOrientationObject.transform.localEulerAngles = Vector3.zero;
				//wheelColliders[i] = new WheelCollider((i < 2) ? settings.frontWheelSettings : settings.rearWheelSettings, wheelObject, wheelOrientationObject);
				wheelObjects[i] = wheelObject;
				wheelOrientationObjects[i] = wheelOrientationObject;
			}

			//Initialize suspension settings
			//suspension = new Suspension(settings.suspensionSettings, gameObject, wheelObjects, wheelOrientationObjects, wheelColliders);

			//drivetrain = new Drivetrain(settings.drivetrainSettings);
			//drivetrain.SetWheels(wheelObjects, wheelColliders);
			//drivetrain.Setup();

			//Add rotating wheel mesh GameObjects to wheel GameObjects

			for (int i = 0; i < 4; i++)
			{
				GameObject visualObject = GameObject.Find(names[i] + "M");
				if (visualObject == null) continue;
				visualObject.transform.parent = wheelObjects[i].transform;
				//wheelColliders[i].SetVisualObject(visualObject, false);
			}

			//Add static wheel mesh GameObjects to wheel GameObjects

			for (int i = 0; i < 4; i++)
			{
				GameObject wheelStaticMesh = GameObject.Find(names[i] + "S");
				if (wheelStaticMesh == null) continue;
				wheelStaticMesh.transform.parent = wheelObjects[i].transform;
				//wheelColliders[i].SetVisualObject(wheelStaticMesh, true);
			}

			//vehicleInputData = GetComponent<VehicleInput>().vehicleInputData;
		}
		
		/*void OnDrawGizmos()
		{
			Gizmos.matrix = transform.localToWorldMatrix;
			Handles.matrix = transform.localToWorldMatrix;

			float pointRadius = 0.01f; //Suspension debug point radius

			for (int i = 0; i < 4; i++)
			{
				Gizmos.color = Color.red;

				Vector3 localAnchor = settings.suspensionSettings.offset;

				localAnchor += (i < 2 ? 1 : -1) * Vector3.forward * settings.suspensionSettings.wheelBase / 2.0f;

				if (i < 2) localAnchor += (i % 2 == 0 ? -1 : 1) * Vector3.right * settings.suspensionSettings.frontWidth;
				else localAnchor += (i % 2 == 0 ? -1 : 1) * Vector3.right * settings.suspensionSettings.rearWidth;

				Gizmos.DrawSphere(localAnchor, pointRadius);

				localAnchor -= Vector3.up * ((i < 2) ? settings.suspensionSettings.frontSpringSettings.height : settings.suspensionSettings.rearSpringSettings.height);

				Gizmos.DrawSphere(localAnchor, pointRadius);

				Vector3 point = localAnchor;

				//Draw minimal length
				Gizmos.color = Color.green;
				Handles.color = Color.green;
				point -= Vector3.up * ((i < 2) ? settings.suspensionSettings.frontSpringSettings.minLength : settings.suspensionSettings.rearSpringSettings.minLength);
				Gizmos.DrawSphere(point, pointRadius);

				//Draw bump stop
				Gizmos.color = Color.yellow;
				point -= Vector3.up * ((i < 2) ? settings.suspensionSettings.frontSpringSettings.bumpStopLength : settings.suspensionSettings.rearSpringSettings.bumpStopLength);
				Gizmos.DrawSphere(point, pointRadius);

				//Draw maximum length
				Gizmos.color = Color.green;
				Handles.color = Color.green;
				point = localAnchor;
				point -= Vector3.up * ((i < 2) ? settings.suspensionSettings.frontSpringSettings.maxLength : settings.suspensionSettings.rearSpringSettings.maxLength);
				Gizmos.DrawSphere(point, pointRadius);

				//Draw rebound stop
				Gizmos.color = Color.magenta;
				point += Vector3.up * ((i < 2) ? settings.suspensionSettings.frontSpringSettings.bumpStopLength : settings.suspensionSettings.rearSpringSettings.bumpStopLength);
				Gizmos.DrawSphere(point, pointRadius);

				//Draw rest length
				Gizmos.color = Color.white;
				point = localAnchor;
				point -= Vector3.up * ((i < 2) ? settings.suspensionSettings.frontSpringSettings.restingLength : settings.suspensionSettings.rearSpringSettings.restingLength);
				Gizmos.DrawSphere(point, pointRadius);

				if (!Application.isPlaying)
				{
					float weightForce = 9.81f * settings.inertiaBoxSettings.mass / 4.0f;

					float a = ((i < 2) ? settings.suspensionSettings.frontSpringSettings.height : settings.suspensionSettings.rearSpringSettings.height);
					float b = ((i < 2) ? settings.suspensionSettings.frontSpringSettings.restingLength : settings.suspensionSettings.rearSpringSettings.restingLength) + a;

					float epsilon = 0.001f;
					float fa, fsr, sr = 0.0f;

					fa = SuspensionJoint.GetForceFromPositionStatic(a, (i < 2) ? settings.suspensionSettings.frontSpringSettings : settings.suspensionSettings.rearSpringSettings) - weightForce;

					while (Mathf.Abs(b - a) > epsilon)
					{
						sr = (a + b) / 2.0f;

						fsr = SuspensionJoint.GetForceFromPositionStatic(sr, (i < 2) ? settings.suspensionSettings.frontSpringSettings : settings.suspensionSettings.rearSpringSettings) - weightForce;
						fa = SuspensionJoint.GetForceFromPositionStatic(a, (i < 2) ? settings.suspensionSettings.frontSpringSettings : settings.suspensionSettings.rearSpringSettings) - weightForce;

						if (Mathf.Abs(fsr) <= epsilon)
							break;
						else if (fsr * fa < 0.0f)
							b = sr;
						else
							a = sr;
					};

					Gizmos.color = Color.cyan;
					point = localAnchor;
					point -= Vector3.up * (sr - ((i < 2) ? settings.suspensionSettings.frontSpringSettings.height : settings.suspensionSettings.rearSpringSettings.height));
					Gizmos.DrawSphere(point, pointRadius);

					DebugDrawWheel(point,
						Vector3.right,
						(i < 2) ? settings.frontWheelSettings.width : settings.rearWheelSettings.width,
						(i < 2) ? settings.frontWheelSettings.radius : settings.rearWheelSettings.radius,
						(i < 2) ? settings.frontWheelSettings.innerRadius : settings.rearWheelSettings.innerRadius
						);
				}
				else if (wheelObjects[i] != null)
				{
					Vector3 position = transform.InverseTransformPoint(wheelOrientationObjects[i].transform.position);
					Vector3 normal = transform.InverseTransformDirection(wheelOrientationObjects[i].transform.right);

					Gizmos.color = Color.cyan;
					Gizmos.DrawSphere(position, pointRadius);

					DebugDrawWheel(position,
						normal,
						(i < 2) ? settings.frontWheelSettings.width : settings.rearWheelSettings.width,
						(i < 2) ? settings.frontWheelSettings.radius : settings.rearWheelSettings.radius,
						(i < 2) ? settings.frontWheelSettings.innerRadius : settings.rearWheelSettings.innerRadius
						);
				}
			}


			Gizmos.color = Color.yellow;
			Gizmos.DrawWireCube(settings.inertiaBoxSettings.offset, settings.inertiaBoxSettings.size);

			Gizmos.DrawWireSphere(settings.inertiaBoxSettings.offset + InertiaBox.CalculateCenterOfMass(settings.inertiaBoxSettings), settings.inertiaBoxSettings.mass * 0.0001f);

			Gizmos.color = Color.cyan;

			Gizmos.matrix = Matrix4x4.TRS(transform.position + transform.rotation * settings.frontWingSettings.position, Quaternion.Euler(transform.rotation.eulerAngles.x - settings.frontWingSettings.angle, transform.rotation.eulerAngles.y, transform.rotation.eulerAngles.z), Vector3.one);
			Gizmos.DrawWireCube(Vector3.zero, new Vector3(settings.frontWingSettings.span, 0f, settings.frontWingSettings.chord));

			Gizmos.matrix = Matrix4x4.TRS(transform.position + transform.rotation * settings.rearWingSettings.position, Quaternion.Euler(transform.rotation.eulerAngles.x - settings.rearWingSettings.angle, transform.rotation.eulerAngles.y, transform.rotation.eulerAngles.z), Vector3.one);
			Gizmos.DrawWireCube(Vector3.zero, new Vector3(settings.rearWingSettings.span, 0f, settings.rearWingSettings.chord));
		}
		*/

		void DebugDrawWheel(Vector3 position, Vector3 normal, float width, float radius, float innerRadius)
		{
			Handles.color = new Color(0f, 1f, 1f, 1f);
			Handles.zTest = UnityEngine.Rendering.CompareFunction.LessEqual;
			Handles.DrawWireDisc(position + normal * width / 2f, normal, radius);
			Handles.DrawWireDisc(position - normal * width / 2f, normal, radius);
			Handles.DrawWireDisc(position + normal * width / 2f, normal, innerRadius);
			Handles.DrawWireDisc(position - normal * width / 2f, normal, innerRadius);

			Handles.color = new Color(0f, 1f, 1f, 0.1f);
			Handles.zTest = UnityEngine.Rendering.CompareFunction.Greater;
			Handles.DrawWireDisc(position + normal * width / 2f, normal, radius);
			Handles.DrawWireDisc(position - normal * width / 2f, normal, radius);
			Handles.DrawWireDisc(position + normal * width / 2f, normal, innerRadius);
			Handles.DrawWireDisc(position - normal * width / 2f, normal, innerRadius);

			Handles.zTest = UnityEngine.Rendering.CompareFunction.Always;
		}

		void FixedUpdate()
		{
			float delta = Time.fixedDeltaTime;

			state.accelerationLS = (vehicleBody.velocity - lastVelocity) / delta;
			lastVelocity = vehicleBody.velocity;
			state.velocityLS = transform.InverseTransformDirection(vehicleBody.velocity);
			state.accelerationLS = transform.InverseTransformDirection(state.accelerationLS);

			//Step(delta);
		}

		/*void Step(float delta)
		{
			frontWing.Step();
			rearWing.Step();

			suspension.Step(delta, vehicleInputData.steer);

			drivetrain.Step(
				delta,
				vehicleInputData.throttle,
				vehicleInputData.clutch,
				vehicleInputData.brake,
				vehicleInputData.handbrake
				);
		}

		*/
		Vector3 lastVelocity = Vector3.zero;
		public Vector3 acceleration = Vector3.zero;

		// void UpdateSteering(float delta)
		// {
		// 	Vector3 localVelocity = transform.InverseTransformDirection(vehicleBody.velocity);

		// 	float mu = 1.4f;
		// 	float g = 9.81f;
		// 	// float radius = velocity.z * velocity.z / g / mu;
		// 	// float radius = Mathf.Abs(localVelocity.z) / g / mu;
		// 	float radius = Mathf.Pow(Mathf.Abs(localVelocity.z), 1.2f) / g / mu;
		// 	float angle = Mathf.Asin(Mathf.Clamp(geometry.wheelBase / radius, -1f, 1f)) * Mathf.Rad2Deg;
		// 	// angle = 360f;

		// 	float maxAngle = 38f;

		// 	float steerAngle = vehicleInputData.steer * maxAngle;
		// 	steerAngle = Mathf.Clamp(steerAngle, -angle, angle);
		// 	steerAngle = Mathf.Clamp(steerAngle, -maxAngle, maxAngle);
		// 	steerAngle = Mathf.Deg2Rad * steerAngle;

		// 	float l =  geometry.wheelBase;
		// 	float w = geometry.frontAxle.width;

		// 	float steerAngleLeft = Mathf.Atan(2f * l * Mathf.Sin(steerAngle) / (2f * l * Mathf.Cos(steerAngle) + w * Mathf.Sin(steerAngle)));
		// 	float steerAngleRight = Mathf.Atan(2f * l * Mathf.Sin(steerAngle) / (2f * l * Mathf.Cos(steerAngle) - w * Mathf.Sin(steerAngle)));

		// 	wheels[0].UpdateSteerAngle(steerAngleLeft * Mathf.Rad2Deg);
		// 	wheels[1].UpdateSteerAngle(steerAngleRight * Mathf.Rad2Deg);
		// }

		// void UpdateSuspension(float delta)
		// {
		// 	geometry.frontAxle.Step(delta);
		// 	geometry.rearAxle.Step(delta);

		// 	for (int i = 0; i < 4; i++) wheels[i].UpdateVerticalForce(delta);
		// }

		// void UpdateWheels(float delta)
		// {
		// 	for (int i = 0; i < 4; i++) wheels[i].Step(delta);
		// }
	}
}