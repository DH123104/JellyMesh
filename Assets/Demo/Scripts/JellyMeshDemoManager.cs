using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class JellyMeshDemoManager : MonoBehaviour 
{
	public Material m_DebugRenderMaterial;
	public Mesh m_DebugMesh;
	public bool m_DrawDebugText = true;

	bool m_DrawPhysicsBodies = false;
	bool m_DebugRenderersCreated = false;
	bool m_KeyboardTogglePhysicsBodies = false;
	GameObject[] m_BlobObjects;

	/// <summary>
	/// Start this instance.
	/// </summary>
	void Start () 
	{
		m_BlobObjects = GameObject.FindGameObjectsWithTag("Blob");
		Physics.gravity = new Vector3(0, -9.8f, 0);
	}

	/// <summary>
	/// Turn all the debug sprite renderers on/off
	/// </summary>
	void SetSpriteRenderersEnabled(bool enabled)
	{
		foreach(GameObject gameObject in m_BlobObjects)
		{
			JellyMesh jellyMesh = gameObject.GetComponent<JellyMesh>();

			if(jellyMesh)
			{
				jellyMesh.GetComponent<MeshRenderer>().enabled = !enabled;

				foreach(JellyMesh.ReferencePoint referencePoint in jellyMesh.ReferencePoints)
				{
					GameObject refPointObject = referencePoint.GameObject;
					
					if(refPointObject && referencePoint.GameObject.transform.childCount > 0)
					{
						referencePoint.GameObject.transform.GetChild(0).gameObject.SetActive(enabled);
					}
				}
			}
		}
	}

	void Update()
	{
		if(Input.GetKeyDown(KeyCode.R))
		{
			m_KeyboardTogglePhysicsBodies = true;
		}
	}
	
	/// <summary>
	/// Draws the GUI
	/// </summary>
	void OnGUI () 
	{
		if(!m_DrawDebugText)
		{
			return;
		}

		GUI.Label(new Rect(20,20,900,20), "Mouse: Look Around");
		GUI.Label(new Rect(20,40,900,20), "W/A/S/D Keys: Move Camera");
		GUI.Label(new Rect(20,60,900,20), "Left Mouse Button: Fire Physics Object");
		GUI.Label(new Rect(20,80,900,20), "R Key: Toggle Physics Rendering");

		if(m_DrawPhysicsBodies)
		{
			if(m_KeyboardTogglePhysicsBodies) 
			{
				m_DrawPhysicsBodies = !m_DrawPhysicsBodies;
				SetSpriteRenderersEnabled(false);
			}
		}
		else
		{
			if(m_KeyboardTogglePhysicsBodies) 
			{
				m_DrawPhysicsBodies = !m_DrawPhysicsBodies;
				
				if(!m_DebugRenderersCreated)
				{
					foreach(GameObject gameObject in m_BlobObjects)
					{
						JellyMesh jellyMesh = gameObject.GetComponent<JellyMesh>();

						if(jellyMesh)
						{
							// Go through and create a simple circle mesh for each body
							foreach(JellyMesh.ReferencePoint referencePoint in jellyMesh.ReferencePoints)
							{
								GameObject referencePointObject = new GameObject("Debug Renderer");
								referencePointObject.transform.parent = referencePoint.GameObject.transform;
								referencePointObject.transform.localPosition = Vector3.zero;
								
								if(referencePointObject)
								{
									MeshRenderer meshRenderer = referencePointObject.AddComponent<MeshRenderer>();
									MeshFilter meshFilter = referencePointObject.AddComponent<MeshFilter>();
									meshFilter.sharedMesh = m_DebugMesh;	
									meshRenderer.sharedMaterial = m_DebugRenderMaterial;
									referencePointObject.transform.localScale = new Vector3(referencePoint.Radius, referencePoint.Radius, referencePoint.Radius);
								}
							}
						}
					}
					
					m_DebugRenderersCreated = true;
				}
				
				SetSpriteRenderersEnabled(true);
			}
		}

		m_KeyboardTogglePhysicsBodies = false;
	}
}
