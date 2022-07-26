using UnityEngine;
using System.Collections;

public class CameraMove : MonoBehaviour 
{
	public float m_MovementForce;
	Rigidbody m_RigidBody;

	// Use this for initialization
	void Start () 
	{	
		m_RigidBody = GetComponent<Rigidbody>();
	}
	
	// Update is called once per frame
	void Update () 
	{
		// Get the input vector from keyboard or analog stick
		Vector3 directionVector = new Vector3(Input.GetAxis("Horizontal"), 0, Input.GetAxis("Vertical"));
		
		if (directionVector != Vector3.zero) 
		{
			// Get the length of the directon vector and then normalize it
			// Dividing by the length is cheaper than normalizing when we already have the length anyway
			float directionLength = directionVector.magnitude;
			directionVector = directionVector / directionLength;
			
			// Make sure the length is no bigger than 1
			directionLength = Mathf.Min(1, directionLength);
			
			// Make the input vector more sensitive towards the extremes and less sensitive in the middle
			// This makes it easier to control slow speeds when using analog sticks
			directionLength = directionLength * directionLength;
			
			// Multiply the normalized direction vector by the modified length
			directionVector = directionVector * directionLength;
		}

		m_RigidBody.AddForce(this.transform.forward * directionVector.z * m_MovementForce);
		m_RigidBody.AddForce(this.transform.right * directionVector.x * m_MovementForce);
	}
}
