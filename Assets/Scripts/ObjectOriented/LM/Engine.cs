using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UDrive
{
	[System.Serializable]
	public class Engine
	{
		public Engine(Settings settings)
		{
			this.settings = settings;
			Setup();
		}
		public const float RPM2AV = 2f * Mathf.PI / 60f;
		public const float AV2RPM = 1f / RPM2AV;
		public float atmPressure = 14.706826606f;
		public float angularVelocity = 0f;
		public float flywheelInertia = 0f;
		float reactionTorque = 0f;
		[HideInInspector]
		public float limiterTimer = 0f;
		public Settings settings { get; private set;}

		public void Setup()
		{
			flywheelInertia = settings.flywheelMass * settings.flywheelRadius * settings.flywheelRadius / 2f;
		}

		public float RawTorque(float rpm, float throttle) // Produces the initial torque of the engine
		{
			float torque = 0f;
			if (rpm >= 0f && rpm < settings.idleRPM) // If rpm is < idle RPM, produce idle torque
			{
				//torque = (idleTorque + FrictionTorque(idleRPM)) / (idleRPM - idleRPM * stallSensitivity) * (rpm - idleRPM * stallSensitivity);
				torque = settings.idleTorque;
			}
			else if (rpm >= settings.idleRPM && rpm < settings.peakRPM) // If in the range of engine RPM
			{
				float scaledRPM = (rpm - settings.idleRPM) / (settings.peakRPM - settings.idleRPM); // Ratio of current RPM/Max RPM
				torque = ((settings.peakTorque - settings.idleTorque + FrictionTorque(settings.peakRPM) - FrictionTorque(settings.idleRPM)) * Curve(scaledRPM, settings.shapeA, settings.shapeB)) + settings.idleTorque + FrictionTorque(settings.idleRPM);
				//			calculate torque
			}
			else if (rpm >= settings.peakRPM)
			{
				torque = ((FrictionTorque(settings.maxRPM) - FrictionTorque(settings.peakRPM) - settings.peakTorque) / Mathf.Pow(settings.maxRPM - settings.peakRPM, 2f) * Mathf.Pow(rpm - settings.peakRPM, 2f)) + settings.peakTorque + FrictionTorque(settings.peakRPM);
			}
			else
			{
				return 0f;
			}
			return Mathf.Max(torque * throttle, 0f);
		}

		public float FrictionTorque(float rpm)
		{
			float friction = settings.frictionTorque + settings.rollingFriction * Mathf.Abs(rpm) * RPM2AV + Mathf.Pow(settings.viscousFriction * rpm * RPM2AV, 2f);
			return friction * Mathf.Sign(rpm);
		}

		public float Curve(float x, float a, float b)
		{
			return 3f * a * x * Mathf.Pow(1f - x, 2f) + 3f * (b + 1) * (1 - x) * Mathf.Pow(x, 2f) + Mathf.Pow(x, 3f);
		}

		public float GetTorque(float rpm, float throttle)
		{
			if(settings.limiterType == Settings.LimiterType.Hard) limiterTimer = angularVelocity * AV2RPM > settings.limitRPM ? settings.limiterTime : limiterTimer;
			return RawTorque(rpm, throttle) * (limiterTimer <= 0f ? 1f : 0f) - FrictionTorque(rpm) + reactionTorque;
		}

		public float totalTorque = 0f;

		public float Step(float delta, float throttle, float realThrottle)
		{
			float idleThrottle = Mathf.Clamp01(settings.idleRPM - Mathf.Abs(angularVelocity) * AV2RPM);
			float actualThrottle = (1f - idleThrottle) * throttle;
			totalTorque = GetTorque(angularVelocity * AV2RPM, actualThrottle + idleThrottle);
			angularVelocity += delta * totalTorque / flywheelInertia;

			if(settings.limiterType == Settings.LimiterType.Hard) limiterTimer = Mathf.Max(limiterTimer - delta, 0f);

			return totalTorque;
		}

		public void AddReactionTorque(float reactionTorque)
		{
			this.reactionTorque = reactionTorque;
		}

		public struct EngineSpecs
		{
			public float limitTorque;
			public float limitRPM;

			public float maxTorque;
			public float maxTorqueRPM;

			public float maxPower;
			public float maxPowerRPM;

			public float idleTorque;
			public float idleRPM;
			
			public float stallRPM;
		}

		[System.Serializable]
		public class Settings
		{
			public float idleRPM = 800f;
			public float idleTorque = 200f;
			public float peakRPM = 6500f;
			public float peakTorque = 244f;
			public float maxRPM = 7000f;
			public float shapeA = 0.0f;
			public float shapeB = 0.0f;
			public float stallSensitivity = 0.5f;
			public float flywheelMass = 20.0f;
			public float flywheelRadius = 0.2f;


			[Header("Engine Friction")]
			public float frictionTorque = 20f;
			[Range(0f, 3f)]
			public float rollingFriction = 0.1f;
			[Range(0f, 0.2f)]
			public float viscousFriction = 0.01f;
			public enum LimiterType { Hard, Soft, Off };
			public LimiterType limiterType;
			public float limiterTime = 0f;
			public float limitRPM = 6000f;

		}
	}
}