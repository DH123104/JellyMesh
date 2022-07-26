using UnityEngine;
using System.Collections;

public class JellyMeshBlobBehaviour : MonoBehaviour 
{
	public float m_MinBounceTime = 0.3f;
	public float m_MaxBounceTime = 1.0f;

	public float m_MinJumpForce = 10.0f;
	public float m_MaxJumpForce = 10.0f;

	public float m_MinTorqueForce = 10.0f;
	public float m_MaxTorqueForce = 10.0f;

	public Vector3 m_MinJumpVector = new Vector3(-0.1f, 1.0f, 0.0f);
	public Vector3 m_MaxJumpVector = new Vector3(0.1f, 1.0f, 0.0f);
	public Vector3 m_MinTorqueVector = new Vector3(-0.1f, -0.1f, -0.1f);
	public Vector3 m_MaxTorqueVector = new Vector3(0.1f, 0.1f, 0.1f);
	public bool m_CentralPointOnly = false;
	
	float m_EyeTimer;
	float m_QuaternionLerpTimer;

	JellyMesh m_JellyMesh;
	Quaternion m_StartEyeRotation;
	Quaternion m_EndEyeRotation;
	Quaternion m_EyeInitialRotation;
	Transform m_EyeLeft;
	Transform m_EyeRight;
	float m_BounceTimer;

	/// <summary>
	/// Start this instance.
	/// </summary>
	void Start () 
	{
		m_JellyMesh = GetComponent<JellyMesh>();
		m_BounceTimer = UnityEngine.Random.Range(m_MinBounceTime, m_MaxBounceTime);
		m_EyeLeft = transform.Find("Eye Left");
		m_EyeRight = transform.Find("Eye Right");

		if(m_EyeLeft)
		{
			m_EyeInitialRotation = m_EyeLeft.localRotation;
		}

		m_StartEyeRotation = m_EyeInitialRotation * Quaternion.Euler(Vector3.zero);
		m_EndEyeRotation = m_EyeInitialRotation * Quaternion.Euler(Vector3.zero);
	}
	
	/// <summary>
	/// Update this instance.
	/// </summary>
	void Update () 
	{
		m_BounceTimer -= Time.deltaTime;

		// Randomly bounce around
		if(m_BounceTimer < 0.0f)
		{
			Vector3 jumpVector = Vector3.zero;
			jumpVector.x = UnityEngine.Random.Range(m_MinJumpVector.x, m_MaxJumpVector.x);
			jumpVector.y = UnityEngine.Random.Range(m_MinJumpVector.y, m_MaxJumpVector.y);
			jumpVector.z = UnityEngine.Random.Range(m_MinJumpVector.z, m_MaxJumpVector.z);
			jumpVector.Normalize();

			Vector3 torqueVector = Vector3.zero;
			torqueVector.x = UnityEngine.Random.Range(m_MinTorqueVector.x, m_MaxTorqueVector.x);
			torqueVector.y = UnityEngine.Random.Range(m_MinTorqueVector.y, m_MaxTorqueVector.y);
			torqueVector.z = UnityEngine.Random.Range(m_MinTorqueVector.z, m_MaxTorqueVector.z);
			torqueVector.Normalize();

			m_JellyMesh.AddForce(jumpVector * UnityEngine.Random.Range(m_MinJumpForce, m_MaxJumpForce), m_CentralPointOnly);
			m_JellyMesh.AddTorque(torqueVector * UnityEngine.Random.Range(m_MinTorqueForce, m_MaxTorqueForce), false);
			m_BounceTimer = UnityEngine.Random.Range(m_MinBounceTime, m_MaxBounceTime);
		}

		m_EyeTimer -= Time.deltaTime;
		m_QuaternionLerpTimer += Time.deltaTime;

		if(m_EyeLeft && m_EyeRight)
		{
			if(m_EyeTimer < 0.0f)
			{
				m_EyeTimer = UnityEngine.Random.Range(2.0f, 3.0f);
				m_QuaternionLerpTimer = 0.0f;

				float randomXAngle = UnityEngine.Random.Range(-45, 45);
				float randomZAngle = UnityEngine.Random.Range(-45, 45);
				m_StartEyeRotation = m_EyeLeft.transform.localRotation;
				m_EndEyeRotation = m_EyeInitialRotation * Quaternion.Euler(randomXAngle, 0.0f, randomZAngle);
			}

			Quaternion lerpedRotation = Quaternion.Lerp(m_StartEyeRotation, m_EndEyeRotation, Mathf.Clamp01(m_QuaternionLerpTimer * 2));

			m_EyeLeft.transform.localRotation = lerpedRotation;
			m_EyeRight.transform.localRotation = lerpedRotation;
		}
	}
}