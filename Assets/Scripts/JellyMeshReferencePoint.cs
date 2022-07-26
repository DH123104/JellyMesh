using UnityEngine;
using System.Collections;

public class JellyMeshReferencePoint : MonoBehaviour 
{
	public GameObject ParentJellyMesh { get; set; }
	public bool SendCollisionMessages { get { return m_SendCollisionMessages; } set { m_SendCollisionMessages = value; } }
	public int Index { get; set; }

	JellyMesh.JellyCollision m_JellyCollision = new JellyMesh.JellyCollision();
	JellyMesh.JellyCollision2D m_JellyCollision2D = new JellyMesh.JellyCollision2D();

	JellyMesh.JellyCollider m_JellyCollider = new JellyMesh.JellyCollider();
	JellyMesh.JellyCollider2D m_JellyCollider2D = new JellyMesh.JellyCollider2D();

	bool m_SendCollisionMessages = true;

	public JellyMeshReferencePoint()
	{
		m_JellyCollision.ReferencePoint = this;
		m_JellyCollision2D.ReferencePoint = this;
		m_JellyCollider.ReferencePoint = this;
		m_JellyCollider2D.ReferencePoint = this;
	}
	
	void OnCollisionEnter(Collision collision)
	{
		if(ParentJellyMesh && SendCollisionMessages)
		{
			m_JellyCollision.Collision = collision;
			ParentJellyMesh.SendMessage("OnJellyCollisionEnter", m_JellyCollision, SendMessageOptions.DontRequireReceiver);
		}
	}

	void OnCollisionEnter2D(Collision2D collision)
	{
		if(ParentJellyMesh && SendCollisionMessages)
		{
			m_JellyCollision2D.Collision2D = collision;
			ParentJellyMesh.SendMessage("OnJellyCollisionEnter2D", m_JellyCollision2D, SendMessageOptions.DontRequireReceiver);
		}
	}

	void OnCollisionExit(Collision collision)
	{
		if(ParentJellyMesh && SendCollisionMessages)
		{
			m_JellyCollision.Collision = collision;
			ParentJellyMesh.SendMessage("OnJellyCollisionExit", m_JellyCollision, SendMessageOptions.DontRequireReceiver);
		}
	}

	void OnCollisionExit2D(Collision2D collision)
	{
		if(ParentJellyMesh && SendCollisionMessages)
		{
			m_JellyCollision2D.Collision2D = collision;
			ParentJellyMesh.SendMessage("OnJellyCollisionExit2D", m_JellyCollision2D, SendMessageOptions.DontRequireReceiver);
		}
	}

	void OnCollisionStay(Collision collision)
	{
		if(ParentJellyMesh && SendCollisionMessages)
		{
			m_JellyCollision.Collision = collision;
			ParentJellyMesh.SendMessage("OnJellyCollisionStay", m_JellyCollision, SendMessageOptions.DontRequireReceiver);
		}
	}

	void OnCollisionStay2D(Collision2D collision)
	{
		if(ParentJellyMesh && SendCollisionMessages)
		{
			m_JellyCollision2D.Collision2D = collision;
			ParentJellyMesh.SendMessage("OnJellyCollisionStay2D", m_JellyCollision2D, SendMessageOptions.DontRequireReceiver);
		}
	}

	void OnTriggerEnter(Collider collider)
	{
		if(ParentJellyMesh && SendCollisionMessages)
		{
			m_JellyCollider.Collider = collider;
			ParentJellyMesh.SendMessage("OnJellyTriggerEnter", m_JellyCollider, SendMessageOptions.DontRequireReceiver);
		}
	}
	
	void OnTriggerEnter2D(Collider2D collider)
	{
		if(ParentJellyMesh && SendCollisionMessages)
		{
			m_JellyCollider2D.Collider2D = collider;
			ParentJellyMesh.SendMessage("OnJellyTriggerEnter2D", m_JellyCollider2D, SendMessageOptions.DontRequireReceiver);
		}
	}
	
	void OnTriggerExit(Collider collider)
	{
		if(ParentJellyMesh && SendCollisionMessages)
		{
			m_JellyCollider.Collider = collider;
			ParentJellyMesh.SendMessage("OnJellyTriggerExit", m_JellyCollider, SendMessageOptions.DontRequireReceiver);
		}
	}
	
	void OnTriggerExit2D(Collider2D collider)
	{
		if(ParentJellyMesh && SendCollisionMessages)
		{
			m_JellyCollider2D.Collider2D = collider;
			ParentJellyMesh.SendMessage("OnJellyTriggerExit2D", m_JellyCollider2D, SendMessageOptions.DontRequireReceiver);
		}
	}
	
	void OnTriggerStay(Collider collider)
	{
		if(ParentJellyMesh && SendCollisionMessages)
		{
			m_JellyCollider.Collider = collider;
			ParentJellyMesh.SendMessage("OnJellyTriggerStay", m_JellyCollider, SendMessageOptions.DontRequireReceiver);
		}
	}
	
	void OnTriggerStay2D(Collider2D collider)
	{
		if(ParentJellyMesh && SendCollisionMessages)
		{
			m_JellyCollider2D.Collider2D = collider;
			ParentJellyMesh.SendMessage("OnJellyTriggerStay2D", m_JellyCollider2D, SendMessageOptions.DontRequireReceiver);
		}
	}
}
