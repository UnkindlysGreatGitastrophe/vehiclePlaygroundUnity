using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Jobs;
using Unity.Collections;
using Unity.Burst;
using UnityEditor;
using System;
using Unity.VisualScripting;

namespace UDrive
{
	[System.Serializable]
	public class WheelCollider
	{
		public WheelCollider(Settings settings, GameObject wheelObject, GameObject wheelOrientationObject)
		{
			this.settings = settings;
			this.wheelObject = wheelObject;
			this.wheelOrientationObject = wheelOrientationObject;
			Setup();
		}

		[System.Serializable]
		public class Settings
		{
			public float mass = 20f,
			radius = 0.3f,
			width = 0.2f,
			innerRadius = 0.2f,
			springRate = 200000f,
			damperRate = 2600f;
			public Grip grip;
		}

		[System.Serializable]
		public class Grip
		{
			public float kx = 5000000f,
			ky = 4200000f,
			uax = 1.17f,
			usx = 0.94f,
			ucx = 1.0f,
			vsxp = 100.0f / 3.6f,
			uay = 1.25f,
			usy = 1.12f,
			ucy = 1.5f,
			vsyp = 100.0f / 3.6f,
			k = 0.1f,
			fzn = 2500f;
		}

		[System.Serializable]
		public class State
		{
			public Vector3 velocityLS = Vector3.zero;
			public float angularVelocity = 0f;
			public float angle = 0f;
			public float contactPatchLength = 0f;
			public Vector3 tyreForce = Vector3.zero;
			public Slip slip = new();
			public Vector2 stress = Vector2.zero;
			public float compression = 0f,
			previousCompression = 0f;
			public float slideFactor = 0f;
			public float currentDriveTorque = 0f,
				currentBrakeTorque = 0f,
				currentReactionTorque = 0f;
		}

		public float inertia = 0f;

		public class Slip
		{
			public float differentialSlipRatio = 0f,
				slipRatio = 0f,
				tanSlipAngle = 0f,
				slipAngle = 0f;
			public Vector2 slipVelocity = Vector2.zero;
		}

		public struct WheelHit
		{
			public bool hasHit;
			public Vector3 normal;
			public Vector3 point;
			public float distance;
		}

		public Settings settings { get; private set;}
		public State state;
		//public Skidmarks skidmarksController;
		public Rigidbody vehicleBody;


		[HideInInspector]
		public float camber = 0f;

		RaycastHit[] hitResults;
		WheelHit wheelHit;

		private GameObject wheelObject;
		private GameObject wheelOrientationObject;
		private Rigidbody wheelBody;
		private SphereCollider wheelHubCollider;
		private GameObject dynamicVisualObject;
		private GameObject staticVisualObject;

		public float peakX = 0f;  
		public float peakY = 0f;

		public void Setup()
		{
			state = new();
			wheelHubCollider = wheelObject.AddComponent<SphereCollider>();
			wheelHubCollider.radius = settings.innerRadius;

			//Set up wheel RigidBody
			wheelBody = wheelObject.AddComponent<Rigidbody>();
			wheelBody.mass = settings.mass;

			//Set up inertia
			inertia = settings.mass * settings.radius * settings.radius / 2f;

			//Initialize wheel hit
			wheelHit = new WheelHit
			{
				hasHit = false,
				normal = Vector3.up,
				point = Vector3.zero,
				distance = 0.0f
			};

			hitResults = new RaycastHit[3];

			peakX = FindPeakSlipLong();
			peakY = FindPeakSlipLat();

			//Debug.Log(peakX + ": " + peakY);
		}

		public void SetVisualObject(GameObject visualObject, bool isStatic)
		{
			visualObject.transform.parent = wheelOrientationObject.transform;
			visualObject.transform.position = wheelOrientationObject.transform.position;
			if (isStatic) staticVisualObject = visualObject;
			else dynamicVisualObject = visualObject;
		}

		public void CollisionStep(float delta)
		{
			DoRaycasts();
			GetAverageCollisionPoint();

			//DoSweepTest();
			CalculateVerticalForce(delta);

			//if(wheelHit.hasHit) camber = -Mathf.Deg2Rad * Vector3.SignedAngle(Vector3.ProjectOnPlane(wheelHit.normal, wheelObject.transform.forward), wheelObject.transform.up, wheelObject.transform.forward);
			//else camber = 0f;
		}

		private void DoSweepTest()
		{
			RaycastHit[] hits = wheelBody.SweepTestAll(-wheelBody.transform.up, settings.radius);

			Vector3 averagePoint = Vector3.zero;
			Vector3 averageNormal = Vector3.zero;
			Vector3 normal;
			Vector3 point;
			float averageDistance = 0f;
			float weightSum = 0f;
			bool isHit = false;

			for (int i = 0; i < hits.Length; i++)
			{
				if (hits[i].distance == 0f) continue;

				isHit = true;

				float weight = settings.radius - hits[i].distance;
				weightSum += weight;

				point = hits[i].point;
				averagePoint += point * weight;

				normal = hits[i].normal;
				averageNormal += normal * weight;

				averageDistance += hits[i].distance * weight;
			}

			if (isHit)
			{
				averagePoint /= weightSum;
				averageNormal = Vector3.Normalize(averageNormal / weightSum);
				averageDistance /= weightSum;

				wheelHit.hasHit = true;
				wheelHit.point = averagePoint;
				wheelHit.normal = averageNormal;
				wheelHit.distance = averageDistance;
			}
			else
			{
				wheelHit.hasHit = false;
			}
		}

		private void DoRaycasts()
		{
			Vector3 pos = wheelOrientationObject.transform.position;
			Vector3 right = wheelOrientationObject.transform.right;
			Vector3 up = wheelOrientationObject.transform.up;

			Physics.Raycast(pos + right * settings.width / 2f, -up, out RaycastHit hitA, settings.radius, 1 << 6);
			Physics.Raycast(pos, -up, out RaycastHit hitB, settings.radius, 1 << 6);
			Physics.Raycast(pos - right * settings.width / 2f, -up, out RaycastHit hitC, settings.radius, 1 << 6);

			hitResults[0] = hitA;
			hitResults[1] = hitB;
			hitResults[2] = hitC;
		}

		public void GetAverageCollisionPoint()
		{
			Vector3 averagePoint = Vector3.zero;
			Vector3 averageNormal = Vector3.zero;
			Vector3 normal;
			Vector3 point;
			float averageDistance = 0f;
			float weightSum = 0f;
			bool isHit = false;

			for (int i = 0; i < hitResults.Length; i++)
			{
				if (hitResults[i].distance == 0f) continue;

				isHit = true;

				float weight = settings.radius - hitResults[i].distance;
				weightSum += weight;

				point = hitResults[i].point;
				averagePoint += point * weight;

				normal = hitResults[i].normal;
				averageNormal += normal * weight;

				averageDistance += hitResults[i].distance * weight;
			}

			if (isHit)
			{
				averagePoint /= weightSum;
				averageNormal = Vector3.Normalize(averageNormal / weightSum);
				averageDistance /= weightSum;

				wheelHit.hasHit = true;
				wheelHit.point = averagePoint;
				wheelHit.normal = averageNormal;
				wheelHit.distance = averageDistance;
			}
			else
			{
				wheelHit.hasHit = false;
			}
		}

		public void CalculateVerticalForce(float delta)
		{
			//If tyre not on ground
			if (!wheelHit.hasHit)
			{
				state.compression = 0f;
				state.previousCompression = 0f;
				state.tyreForce.y = 0f;
				state.slideFactor = 0f;
				return;
			}

			// Vertical force
			state.compression = Mathf.Clamp(settings.radius - wheelHit.distance, 0f, settings.radius);
			float compressionVelocity = (state.previousCompression - state.compression) / delta;
			float springForce = state.compression * settings.springRate;
			float damperForce = compressionVelocity * settings.damperRate;
			damperForce = Mathf.Clamp(damperForce, -Mathf.Abs(springForce), Mathf.Abs(springForce));
			state.tyreForce.y = springForce - damperForce;
			state.previousCompression = state.compression;
		}
		float abs = 0f,
			absbrake = 0f,
			absVelo = 0f;

		public void AddTorque(float driveTorque, float brakeTorque, float handbrakeTorque, float delta)
		{
			float fzn = settings.grip.fzn;
			float ul = (1f - settings.grip.k * (state.tyreForce.y - fzn) / fzn);
			float fz = state.tyreForce.y * ul;

			fz = Mathf.Clamp(fz, 0f, fzn * 4f);

						float uax = settings.grip.uax;
			float usx = settings.grip.usx;

			float ux = ul * (uax + (usx - uax) * (Mathf.Abs(state.slip.slipVelocity.x) < settings.grip.vsxp ? Mathf.Exp(-Mathf.Pow(settings.grip.ucx * Mathf.Log10(Mathf.Abs(state.slip.slipVelocity.x / settings.grip.vsxp)), 2f)) : 1f));

			float uay = settings.grip.uay;
			float usy = settings.grip.usy;

			float uy = ul * (uay + (usy - uay) * (Mathf.Abs(state.slip.slipVelocity.y) < settings.grip.vsyp ? Mathf.Exp(-Mathf.Pow(settings.grip.ucy * Mathf.Log10(Mathf.Abs(state.slip.slipVelocity.y / settings.grip.vsyp)), 2f)) : 1f));

			float kx = settings.grip.kx;
			float ky = settings.grip.ky;

			float peakSlipRatio = 9f * fz * fz * ux * ux * uy * uy - 4f * Mathf.Pow(state.contactPatchLength / 2f, 4f) * ky * ky * ux * ux * Mathf.Pow(Mathf.Tan(state.slip.slipAngle), 2f);
			peakSlipRatio = Mathf.Sqrt(Mathf.Abs(peakSlipRatio));
			peakSlipRatio /= (2f * Mathf.Pow(state.contactPatchLength / 2f, 4f) * kx * uy);
			
			abs += Mathf.Sign(Mathf.Abs(state.slip.slipRatio) - peakX);
			abs = Mathf.Clamp01(abs);
			absbrake = Mathf.SmoothDamp(absbrake, abs, ref absVelo, 0.02f);
			//Debug.Log(absbrake);
			state.currentDriveTorque = driveTorque;

			state.currentBrakeTorque = -Mathf.Max(brakeTorque * (1f - absbrake), handbrakeTorque);

			float preBrkAV = state.angularVelocity + delta * (state.currentDriveTorque + state.currentReactionTorque) / inertia;
			float postBrkAV = preBrkAV + delta * state.currentBrakeTorque * Mathf.Sign(preBrkAV) / inertia;

			if (Mathf.Sign(preBrkAV) != Mathf.Sign(postBrkAV))
			{
				state.currentBrakeTorque = -inertia * Mathf.Abs(preBrkAV) / delta;
			}

			state.currentBrakeTorque *= Mathf.Sign(state.angularVelocity);
		}

		public void Step(float delta)
		{
			UpdateAngularVelocity(delta);
			CalculateSlip(delta);
			CalculateFrictionForces();
			CalculateStress();
			ApplyForces();
		}

		void CalculateSlip(float delta)
		{
			if (!wheelHit.hasHit)
			{
				state.slip.differentialSlipRatio = 0f;
				state.slip.slipRatio = 0f;
				state.slip.tanSlipAngle = 0f;
				state.slip.slipAngle = 0f;
				state.slip.slipVelocity = Vector2.zero;
				state.tyreForce.x = 0f;
				state.tyreForce.y = 0f;
				state.slideFactor = 0f;
				return;
			}

			state.velocityLS = wheelOrientationObject.transform.InverseTransformDirection(wheelBody.GetPointVelocity(wheelHit.point));
			state.slip.slipVelocity.x = state.angularVelocity * (settings.radius - state.compression) - state.velocityLS.z;
			state.slip.slipVelocity.y = state.velocityLS.x;

			float maxdampedRoadVeloA = Mathf.Max(Mathf.Abs(state.velocityLS.z), 2f);
			float maxdampedRoadVeloB = Mathf.Max(Mathf.Abs(state.velocityLS.z), 8f);

			float relLon = 0.2f;
			float relLat = 0.6f;
			//float tau = 0.02f;
			float relLonFactor = relLon * Mathf.Max(1f / (Mathf.Abs(state.velocityLS.z) + 1), 1.0f);
			float relLatFactor = relLat * Mathf.Max(1f / (Mathf.Abs(state.velocityLS.z) + 1), 1.0f);

			// float deltaSlipRatio = state.angularVelocity * (settings.radius - state.compression) - state.velocityLS.z - maxdampedRoadVeloA * state.slip.differentialSlipRatio;
			float deltaSlipRatio = state.slip.slipVelocity.x - maxdampedRoadVeloA * state.slip.differentialSlipRatio;
			// deltaSlipRatio -= prevDeltaSlipRatio * 0.5f;
			// prevDeltaSlipRatio = deltaSlipRatio;
			//deltaSlipRatio /= relLonFactor;
	
			deltaSlipRatio /= relLon;
			state.slip.differentialSlipRatio += deltaSlipRatio * delta;
			state.slip.slipRatio = Mathf.Clamp(state.slip.differentialSlipRatio, -100.0f, 100.0f);
			
			// float deltaSlipAngle = -state.velocityLS.x - maxdampedRoadVeloB * state.slip.tanSlipAngle;
			float deltaSlipAngle = -state.slip.slipVelocity.y - maxdampedRoadVeloB * state.slip.tanSlipAngle;
			// deltaSlipAngle -= prevDeltaSlipAngle * 0.5f;
			// prevDeltaSlipAngle = deltaSlipAngle;
			//deltaSlipAngle /= relLatFactor;
			deltaSlipAngle /= relLat;
			state.slip.tanSlipAngle += deltaSlipAngle * delta;

			state.slip.slipAngle = Mathf.Atan(state.slip.tanSlipAngle);
		}

		float prevDeltaSlipRatio = 0f;
		float prevDeltaSlipAngle = 0f;

    float CriticalPoint(float a, float fz, float uy, float ux, float ky, float sy, float kx, float sx)
    {
        float term1 = 216f * Mathf.Sqrt(Mathf.Pow(a, 10f) * Mathf.Pow(fz, 2f) * Mathf.Pow(uy, 2f) * Mathf.Pow(ux, 2f) * (Mathf.Pow(ky, 2f) * Mathf.Pow(sy, 2f) * Mathf.Pow(ux, 2f) + Mathf.Pow(kx, 2f) * Mathf.Pow(sx, 2f) * Mathf.Pow(uy, 2f))) / (5f * Mathf.Pow(fz, 2f) * Mathf.Pow(uy, 2f) * Mathf.Pow(ux, 2f));
        float term2 = 20f * Mathf.Pow(a, 3f);
        float term3 = Mathf.Sqrt(32f * Mathf.Pow(a, 6f) + Mathf.Pow((216f * Mathf.Sqrt(Mathf.Pow(a, 10f) * Mathf.Pow(fz, 2f) * Mathf.Pow(uy, 2f) * Mathf.Pow(ux, 2f) * (Mathf.Pow(ky, 2f) * Mathf.Pow(sy, 2f) * Mathf.Pow(ux, 2f) + Mathf.Pow(kx, 2f) * Mathf.Pow(sx, 2f) * Mathf.Pow(uy, 2f)))) / (5f * Mathf.Pow(fz, 2f) * Mathf.Pow(uy, 2f) * Mathf.Pow(ux, 2f)) - 20f * Mathf.Pow(a, 3f), 2f));
        
        float result = Mathf.Pow(term1 - term2 + term3, 1f / 3f) / (3f * Mathf.Pow(2f, 1f / 3f)) - (2f * Mathf.Pow(2f, 1f / 3f) * Mathf.Pow(a, 2f)) / (3f * Mathf.Pow((216f * Mathf.Sqrt(Mathf.Pow(a, 10f) * Mathf.Pow(fz, 2f) * Mathf.Pow(uy, 2f) * Mathf.Pow(ux, 2f) * (Mathf.Pow(ky, 2f) * Mathf.Pow(sy, 2f) * Mathf.Pow(ux, 2f) + Mathf.Pow(kx, 2f) * Mathf.Pow(sx, 2f) * Mathf.Pow(uy, 2f)))) / (5f * Mathf.Pow(fz, 2f) * Mathf.Pow(uy, 2f) * Mathf.Pow(ux, 2f)) - 20f * Mathf.Pow(a, 3f) + Mathf.Sqrt(32f * Mathf.Pow(a, 6f) + Mathf.Pow((216f * Mathf.Sqrt(Mathf.Pow(a, 10f) * Mathf.Pow(fz, 2f) * Mathf.Pow(uy, 2f) * Mathf.Pow(ux, 2f) * (Mathf.Pow(ky, 2f) * Mathf.Pow(sy, 2f) * Mathf.Pow(ux, 2f) + Mathf.Pow(kx, 2f) * Mathf.Pow(sx, 2f) * Mathf.Pow(uy, 2f)))) / (5f * Mathf.Pow(fz, 2f) * Mathf.Pow(uy, 2f) * Mathf.Pow(ux, 2f)) - 20f * Mathf.Pow(a, 3f), 2f)), 1f / 3f)) - a / 3f;

        return result;
    }

		public void CalculateFrictionForces()
		{
			if (state.tyreForce.y <= 0f)
			{
				state.tyreForce.x = 0f;
				state.tyreForce.z = 0f;
				state.slideFactor = 0f;
				return;
			}

			float fzn = settings.grip.fzn;
			float ul = (1f - settings.grip.k * (state.tyreForce.y - fzn) / fzn);
			float fz = state.tyreForce.y * ul;

			fz = Mathf.Clamp(fz, 0f, fzn * 4f);

			float kx = settings.grip.kx;
			float ky = settings.grip.ky;
			float a = Mathf.Sqrt(settings.radius * settings.radius - (settings.radius - state.compression) * (settings.radius - state.compression));
			state.contactPatchLength = 2f * a;

			float uax = settings.grip.uax;
			float usx = settings.grip.usx;

			float ux = ul * (uax + (usx - uax) * (Mathf.Abs(state.slip.slipVelocity.x) < settings.grip.vsxp ? Mathf.Exp(-Mathf.Pow(settings.grip.ucx * Mathf.Log10(Mathf.Abs(state.slip.slipVelocity.x / settings.grip.vsxp)), 2f)) : 1f));

			float uay = settings.grip.uay;
			float usy = settings.grip.usy;

			float uy = ul * (uay + (usy - uay) * (Mathf.Abs(state.slip.slipVelocity.y) < settings.grip.vsyp ? Mathf.Exp(-Mathf.Pow(settings.grip.ucy * Mathf.Log10(Mathf.Abs(state.slip.slipVelocity.y / settings.grip.vsyp)), 2f)) : 1f));

			float sx = state.slip.slipRatio;
			float sy = Mathf.Tan(state.slip.slipAngle);

			if ((sx == 0f || sy == 0f) || fz <= 0f)
			{
				state.tyreForce.z = 0f;
				state.tyreForce.x = 0f;
				return;
			}

			float xc = CriticalPoint(a, fz, uy, ux, ky, sy, kx, sx);

			float fax = 0f,
				fsx = 0f,
				fay = 0f,
				fsy = 0f;

			float sc = Mathf.Sqrt(sx * sx + sy * sy);

			if(xc < a)
			{
				fax = kx * sx * Mathf.Pow(a - xc, 2f) / 2f;
				fay = ky * sy * Mathf.Pow(a - xc, 2f) / 2f;

				fsx = (1.0f / 8.0f) * (-Mathf.Pow(xc, 5) / Mathf.Pow(a, 5) + (5 * xc) / a + 4);

				fsy = fsx;

				fsx *= fz * ux;
				fsy *= fz * uy;
			}
			else
			{
				fsx = fz * ux;
				fsy = fz * uy;
			}

			fsx *= Mathf.Sign(sx) * Mathf.Sin(Mathf.Atan(Mathf.Abs(sx * uy / sy / ux)));
			fsy *= Mathf.Sign(sy) * Mathf.Sin(Mathf.Atan(Mathf.Abs(sy * ux / sx / uy)));

			state.tyreForce.z = (fsx + fax);
			state.tyreForce.x = (fsy + fay);

			state.slideFactor = (xc + a) / (2f * a);
		}

		void ApplyForces()
		{
			float rollingResistance = 3.2f * state.velocityLS.z * state.contactPatchLength * 4f;
			rollingResistance = 0f;

			Vector3 normal = Vector3.Normalize(wheelHit.normal);

			Vector3 totalForce = wheelOrientationObject.transform.right * state.tyreForce.x
			+ state.tyreForce.y * normal
			+ state.tyreForce.z * wheelOrientationObject.transform.forward
			- rollingResistance * wheelOrientationObject.transform.forward;

			state.currentReactionTorque = -state.tyreForce.z * (settings.radius - state.compression);

			wheelBody.AddForceAtPosition(totalForce, wheelHit.point);
		}

		void CalculateStress()
		{
			if (wheelHit.hasHit)
			{
				state.stress.x = (1f - Mathf.Cos(state.slip.slipAngle)) * 0.1f * Mathf.Abs(state.velocityLS.x);
				state.stress.y = Mathf.Min(Mathf.Abs(state.slip.slipRatio), 1f) * 0.1f * Mathf.Abs(state.angularVelocity * (settings.radius - state.compression) - state.velocityLS.z);
			}
			else state.stress = Vector2.zero;
		}

		public void UpdateAngularVelocity(float delta)
		{
			state.angularVelocity += delta * (state.currentDriveTorque + state.currentBrakeTorque + state.currentReactionTorque) / inertia;

			state.angle += state.angularVelocity * delta;
			state.angle = Mathf.Repeat(state.angle, 2.0f * Mathf.PI);
			dynamicVisualObject.transform.localRotation = Quaternion.Euler(state.angle * Mathf.Rad2Deg, 0.0f, 0.0f);
		}

		public float FindPeakSlipLong()
		{
			float ra = 0f;
			float normalPatchLength = Mathf.Sqrt(Mathf.Pow(settings.radius, 2f) - Mathf.Pow(settings.radius - 3500f / settings.springRate, 2f));
			float rb = 3f * 3500f * settings.grip.uax / (2f * Mathf.Pow(normalPatchLength, 2f) * settings.grip.kx);

			float max = 0f;
			float peak = 0f;

			for (int i = 0; i < 240; i++)
			{
				float s = ra + (float)i / 240f * rb;

				float fz = 3500f;

				float kx = settings.grip.kx;
				float ky = settings.grip.ky;
				float a = normalPatchLength;

				float uax = settings.grip.uax;
				float usx = settings.grip.usx;

				float uay = settings.grip.uay;
				float usy = settings.grip.usy;

				float sx = s;

				if (sx == 0f)
				{
					continue;
				}

				float xc = kx * kx * sx * sx * uay * uay;
				xc = Mathf.Sqrt(xc);
				xc *= 4f * a * a * a;
				xc /= (3f * fz * uax * uay);
				xc -= a;

				float fax = 0f,
					fsx = 0f;

				float sc = Mathf.Sqrt(sx * sx / usx / usx);

				if (xc < a)
				{
					fax = kx * sx / 2f;
					fax = fax * (a - xc) * (a - xc);

					fsx = (a + xc) * (a + xc);
					fsx *= a * (3f * usx + 5f * uax) - xc * (usx + 3f * uax);
					fsx /= 16f * a * a * a;
					fsx *= fz * sx / usx / sc;
				}
				else
				{
					fax = 0f;

					fsx = a * uax + usx * xc;
					fsx /= a + xc;
					fsx *= fz * sx / usx / sc;
				}

				if(fax + fsx > max)
				{
					max = fax + fsx;
					peak = s;
				}
			}

			return peak;
		}

		public float FindPeakSlipLat()
		{
			float ra = 0f;
			float normalPatchLength = Mathf.Sqrt(Mathf.Pow(settings.radius, 2f) - Mathf.Pow(settings.radius - 3500f / settings.springRate, 2f));
			float rb = 3f * 3500f * settings.grip.uay / (2f * Mathf.Pow(normalPatchLength, 2f) * settings.grip.ky);

			float max = 0f;
			float peak = 0f;

			for (int i = 0; i < 240; i++)
			{
				float s = ra + (float)i / 240f * rb;

			float fz = 3500f;

			float kx = settings.grip.kx;
			float ky = settings.grip.ky;
			float a = normalPatchLength;

			float uax = settings.grip.uax;
			float usx = settings.grip.usx;

			float uay = settings.grip.uay;
			float usy = settings.grip.usy;

			float sx = state.slip.slipRatio;
			float sy = s;

			if (sy == 0f)
			{
				continue;
			}

			float xc = ky * ky * sy * sy * uax * uax;
			xc = Mathf.Sqrt(xc);
			xc *= 4f * a * a * a;
			xc /= (3f * fz * uax * uay);
			xc -= a;

			float fay = 0f,
				fsy = 0f;

			float sc = Mathf.Sqrt(sy * sy / usy / usy);

			if (xc < a)
			{
				fay = ky * sy / 2f;
				fay = fay * (a - xc) * (a - xc);

				fsy = (a + xc) * (a + xc);
				fsy *= a * (3f * usy + 5f * uay) - xc * (usy + 3f * uay);
				fsy /= 16f * a * a * a;
				fsy *= fz * sy / usy / sc;
			}
			else
			{
				fay = 0f;

				fsy = a * uay + usy * xc;
				fsy /= a + xc;
				fsy *= fz * sy / usy / sc;
			}

				if(fay + fsy > max)
				{
					max = fay + fsy;
					peak = s;
				}
			}

			return peak;
		}
	}
}