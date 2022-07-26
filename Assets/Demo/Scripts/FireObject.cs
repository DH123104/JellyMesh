using UnityEngine;
using System.Collections;

public class FireObject : MonoBehaviour 
{
	public GameObject m_Object;
	public float m_Force = 1.0f;

	// Update is called once per frame
	void Update () 
	{	
		if(Input.GetMouseButtonDown(0))
		{
			GameObject newObject = GameObject.Instantiate(m_Object) as GameObject;
			newObject.transform.position = this.transform.position + this.transform.forward;
			newObject.GetComponent<Rigidbody>().AddForce(this.transform.forward * m_Force);
			Destroy(newObject, 5.0f);
		}
	}
}
