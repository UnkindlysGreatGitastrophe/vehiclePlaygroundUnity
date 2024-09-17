using System;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting.Antlr3.Runtime;
using UnityEngine;

namespace UDrive
{
	[System.Serializable]
	public class Clutch
	{
		public Clutch(Settings settings)
		{
			this.settings = settings;
		}

		[System.Serializable]
		public class Settings
		{
			public float maxTorque;
			// public float springStiffness;
			// public float damping;
		}

		public Settings settings { get; private set;}

		// float slip = 0f;
		// float angleDiff = 0f;

		public float GetClutchDrag(float clutch, float torqueA, float torqueB, float inertiaA, float inertiaB, float velocityA, float velocityB, float delta)
		{
			float dragLimit = (1f - clutch) * settings.maxTorque;

			float a = torqueB * inertiaA;
			float b = torqueA * inertiaB;
			float c = inertiaA * inertiaB * (velocityA - velocityB) / delta;

			return Mathf.Clamp((a - b + c) / (inertiaA + inertiaB), -dragLimit, dragLimit);
		}

		// public float GetClutchDragB(float clutch, float torqueA, float torqueB, float inertiaA, float inertiaB, float velocityA, float velocityB, float delta)
		// {
		// 	float dragLimit = (1f - clutch) * settings.maxTorque;

		// 	float clutchTorque = inertiaA * velocityA + inertiaB * velocityB + delta * (torqueA + torqueB);
		// 	clutchTorque /= inertiaA + inertiaB;
		// 	clutchTorque -= velocityB;
		// 	clutchTorque *= inertiaB;
		// 	clutchTorque /= delta;

		// 	//float limitedDrag = Mathf.Clamp(clutchTorque, -dragLimit, dragLimit);

		// 	//float slip = Mathf.Abs(clutchTorque) / Mathf.Max(Mathf.Abs(limitedDrag), Mathf.Epsilon);
			
		// 	angleDiff += (velocityA - velocityB) * delta * (1f - clutch);
		// 	angleDiff *= (1f - clutch);

		// 	//float clutchTorqueLimited = Mathf.Clamp(clutchTorque, -dragLimit, dragLimit);

		// 	//float torqueDelta = clutchTorqueLimited - torqueB;


		// 	return Mathf.Clamp(angleDiff * 400.0f + (velocityA - velocityB) * (1f - clutch) * 0.5f, -dragLimit, dragLimit);
		// 	//return Mathf.Clamp(clutchTorque, -dragLimit, dragLimit);
		// }
	}
}