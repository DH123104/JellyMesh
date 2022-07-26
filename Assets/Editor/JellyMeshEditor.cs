using UnityEditor;
using UnityEngine;
using System;

[CustomEditor(typeof(JellyMesh)), CanEditMultipleObjects]
public class JellyMeshEditor : Editor 
{
	public SerializedProperty m_SourceMesh;
	public SerializedProperty m_PhysicsStyle;
	public SerializedProperty m_PhysicsMode;
	public SerializedProperty m_CircleRadiusPoints;
	public SerializedProperty m_ColliderRadius;
	public SerializedProperty m_Stiffness;
	public SerializedProperty m_Mass;
	public SerializedProperty m_DampingRatio;
	public SerializedProperty m_PhysicsMaterial2D;
	public SerializedProperty m_PhysicsMaterial;
    public SerializedProperty m_InterpolationMode;
    public SerializedProperty m_InterpolationMode2D;
    public SerializedProperty m_CollisionDetectionMode;
    public SerializedProperty m_CollisionDetectionMode2D;
	public SerializedProperty m_DistanceExponent;
	public SerializedProperty m_MassStyle;
	public SerializedProperty m_GravityScale;
    public SerializedProperty m_UseGravity;
	public SerializedProperty m_Drag;
	public SerializedProperty m_AngularDrag;
	public SerializedProperty m_SoftBodyScale;
	public SerializedProperty m_SoftBodyOffset;
	public SerializedProperty m_SoftBodyPivotOffset;
	public SerializedProperty m_SoftBodyRotation;
	public SerializedProperty m_MeshScale;
	public SerializedProperty m_AttachPoints;
	public SerializedProperty m_NumAttachPoints;
	public SerializedProperty m_CollideConnected;
	public SerializedProperty m_AttachNeighbors;
	public SerializedProperty m_LockRotationX;
	public SerializedProperty m_LockRotationY;
	public SerializedProperty m_LockRotationZ;
    public SerializedProperty m_LockPositionX;
    public SerializedProperty m_LockPositionY;
    public SerializedProperty m_LockPositionZ;
    public SerializedProperty m_CentralBodyKinematic;
	public SerializedProperty m_RecalculateNormals;

	UnityEngine.Object m_InitialMesh;
	bool m_InitialLockRotationX;
	bool m_InitialLockRotationY;
	bool m_InitialLockRotationZ;
    bool m_InitialLockPositionX;
    bool m_InitialLockPositionY;
    bool m_InitialLockPositionZ;
	bool m_InitialCentralBodyKinematic;
	int m_InitialMassStyle;
    int m_InitialInterpolation2D;
    int m_InitialInterpolation;
    int m_InitialCollisionDetectionMode;
    int m_InitialCollisionDetectionMode2D;
	int m_InitialNumAttachPoints;
	float m_InitialStiffness; 
	float m_InitialDamping;
	float m_InitialMass;
	float m_InitialDistanceExponent;
	float m_InitialGravityScale;
    bool m_InitialUseGravity;
	float m_InitialDrag;
	float m_InitialAngularDrag;
	Vector3 m_InitialMeshScale;

	protected virtual void OnEnable () 
	{
		// Setup the SerializedProperties
		m_SourceMesh = serializedObject.FindProperty("m_SourceMesh");
		m_PhysicsStyle = serializedObject.FindProperty("m_Style");
		m_PhysicsMode = serializedObject.FindProperty("m_2DMode");
		m_CircleRadiusPoints = serializedObject.FindProperty("m_RadiusPoints");
		m_ColliderRadius = serializedObject.FindProperty("m_SphereRadius");
		m_Stiffness = serializedObject.FindProperty("m_Stiffness");
		m_Mass = serializedObject.FindProperty("m_Mass");
		m_DampingRatio = serializedObject.FindProperty("m_DampingRatio");
		m_PhysicsMaterial = serializedObject.FindProperty("m_PhysicsMaterial");
		m_PhysicsMaterial2D = serializedObject.FindProperty("m_PhysicsMaterial2D");
        m_InterpolationMode = serializedObject.FindProperty("m_Interpolation");
        m_InterpolationMode2D = serializedObject.FindProperty("m_Interpolation2D");
        m_CollisionDetectionMode = serializedObject.FindProperty("m_CollisionDetectionMode");
        m_CollisionDetectionMode2D = serializedObject.FindProperty("m_CollisionDetectionMode2D");
		m_DistanceExponent = serializedObject.FindProperty("m_DistanceExponent");
		m_MassStyle = serializedObject.FindProperty("m_MassStyle");
		m_GravityScale = serializedObject.FindProperty("m_GravityScale");
        m_UseGravity = serializedObject.FindProperty("m_UseGravity");
		m_Drag = serializedObject.FindProperty("m_Drag");
		m_AngularDrag = serializedObject.FindProperty("m_AngularDrag");
		m_SoftBodyScale = serializedObject.FindProperty("m_SoftBodyScale");
		m_SoftBodyOffset = serializedObject.FindProperty("m_SoftBodyOffset");
		m_SoftBodyPivotOffset = serializedObject.FindProperty("m_SoftBodyPivotOffset");
		m_SoftBodyRotation = serializedObject.FindProperty("m_SoftBodyRotation");
		m_MeshScale = serializedObject.FindProperty("m_MeshScale");
		m_AttachPoints = serializedObject.FindProperty("m_AttachPoints");
		m_NumAttachPoints = serializedObject.FindProperty("m_NumAttachPoints");
		m_CollideConnected = serializedObject.FindProperty("m_CollideConnected");
		m_AttachNeighbors = serializedObject.FindProperty("m_AttachNeighbors");
		m_LockRotationX = serializedObject.FindProperty("m_LockRotationX");
		m_LockRotationY = serializedObject.FindProperty("m_LockRotationY");
		m_LockRotationZ = serializedObject.FindProperty("m_LockRotationZ");
        m_LockPositionX = serializedObject.FindProperty("m_LockPositionX");
        m_LockPositionY = serializedObject.FindProperty("m_LockPositionY");
        m_LockPositionZ = serializedObject.FindProperty("m_LockPositionZ");
		m_CentralBodyKinematic = serializedObject.FindProperty("m_CentralBodyKinematic");
		m_RecalculateNormals = serializedObject.FindProperty("m_RecalculateNormals");
	}
	
	protected virtual void StoreInitialValues()
	{
		m_InitialMesh = m_SourceMesh.objectReferenceValue;
		m_InitialMassStyle = m_MassStyle.enumValueIndex;
		m_InitialStiffness = m_Stiffness.floatValue;
        m_InitialInterpolation2D = m_InterpolationMode2D.enumValueIndex;
        m_InitialInterpolation = m_InterpolationMode.enumValueIndex;
        m_InitialCollisionDetectionMode = m_CollisionDetectionMode.enumValueIndex;
        m_InitialCollisionDetectionMode2D = m_CollisionDetectionMode2D.enumValueIndex;
		m_InitialDamping = m_DampingRatio.floatValue;
		m_InitialMass = m_Mass.floatValue;
		m_InitialDistanceExponent = m_DistanceExponent.floatValue;
		m_InitialGravityScale = m_GravityScale.floatValue;
        m_InitialUseGravity = m_UseGravity.boolValue;
		m_InitialDrag = m_Drag.floatValue;
		m_InitialAngularDrag = m_AngularDrag.floatValue;
		m_InitialMeshScale = m_MeshScale.vector3Value;
		m_InitialNumAttachPoints = m_NumAttachPoints.intValue;
		m_InitialLockRotationX = m_LockRotationX.boolValue;
		m_InitialLockRotationY = m_LockRotationY.boolValue;
		m_InitialLockRotationZ = m_LockRotationZ.boolValue;
        m_InitialLockPositionX = m_LockPositionX.boolValue;
        m_InitialLockPositionY = m_LockPositionY.boolValue;
        m_InitialLockPositionZ = m_LockPositionZ.boolValue;
		m_InitialCentralBodyKinematic = m_CentralBodyKinematic.boolValue;
	}

	protected virtual void CheckForObjectChanges()
	{
		serializedObject.ApplyModifiedProperties();

		// Update the visible mesh if the mesh or vertex density was changed
		if(m_InitialMeshScale != m_MeshScale.vector3Value ||
		   m_InitialMesh != m_SourceMesh.objectReferenceValue)
		{
			foreach(UnityEngine.Object targetObject in targets)
			{
				JellyMesh targetObjectMesh = targetObject as JellyMesh;
				targetObjectMesh.RefreshMesh();
			}
		}

		// Update the springs if we altered any of their settings
		if(m_InitialStiffness != m_Stiffness.floatValue || m_InitialDamping != m_DampingRatio.floatValue)
		{
			foreach(UnityEngine.Object targetObject in targets)
			{
				JellyMesh targetObjectMesh = targetObject as JellyMesh;
				targetObjectMesh.UpdateJoints();
				targetObjectMesh.WakeUp();
			}
		}
		
		// Recalculate weighting values if the exponent changes
		if(m_InitialDistanceExponent != m_DistanceExponent.floatValue)
		{
			foreach(UnityEngine.Object targetObject in targets)
			{
				JellyMesh targetObjectMesh = targetObject as JellyMesh;
				targetObjectMesh.CalculateWeightingValues();
			}
		}
		
		// Update the mass of each body if the value changed
		if(m_InitialMassStyle != m_MassStyle.enumValueIndex ||
		   m_InitialMass != m_Mass.floatValue ||
		   m_InitialGravityScale != m_GravityScale.floatValue ||
           m_InitialUseGravity != m_UseGravity.boolValue ||
           m_InitialInterpolation != m_InterpolationMode.enumValueIndex ||
           m_InitialInterpolation2D != m_InterpolationMode2D.enumValueIndex ||
           m_InitialCollisionDetectionMode != m_CollisionDetectionMode.enumValueIndex ||
           m_InitialCollisionDetectionMode2D != m_CollisionDetectionMode2D.enumValueIndex ||
		   m_InitialAngularDrag != m_AngularDrag.floatValue ||
		   m_InitialDrag != m_Drag.floatValue)
		{
			foreach(UnityEngine.Object targetObject in targets)
			{
				JellyMesh targetObjectMesh = targetObject as JellyMesh;
				targetObjectMesh.InitMass();
				targetObjectMesh.WakeUp();
			}
		}

		if(m_InitialNumAttachPoints != m_NumAttachPoints.intValue)
		{
			foreach(UnityEngine.Object targetObject in targets)
			{
				JellyMesh targetObjectMesh = targetObject as JellyMesh;
				targetObjectMesh.ResizeAttachPoints();
			}
		}

		if(m_InitialLockRotationX != m_LockRotationX.boolValue ||
		   m_InitialLockRotationY != m_LockRotationY.boolValue ||
		   m_InitialLockRotationZ != m_LockRotationZ.boolValue ||
           m_InitialLockPositionX != m_LockPositionX.boolValue ||
           m_InitialLockPositionY != m_LockPositionY.boolValue ||
           m_InitialLockPositionZ != m_LockPositionZ.boolValue ||
		   m_InitialCentralBodyKinematic != m_CentralBodyKinematic.boolValue)
		{
			foreach(UnityEngine.Object targetObject in targets)
			{
				JellyMesh targetObjectMesh = targetObject as JellyMesh;
				targetObjectMesh.UpdateRotationLock();
			}
		}
	}

	protected virtual void DisplayInspectorGUI()
	{
		JellyMesh targetObject = this.target as JellyMesh;
		EditorGUILayout.PropertyField(m_SourceMesh, new GUIContent("Mesh"));

		if(Application.isPlaying)
		{
			GUI.enabled = false;
		}

		EditorGUILayout.PropertyField(m_MeshScale, new GUIContent("Mesh Scale"));
		EditorGUILayout.PropertyField(m_RecalculateNormals, new GUIContent("Recalculate Normals"));

		GUI.enabled = true;

		if(Application.isPlaying)
		{
			GUI.enabled = false;
		}

		GUILayout.Space(15);

		EditorGUILayout.PropertyField(m_PhysicsMode, new GUIContent("Use 2D Physics"));

		JellyMesh.PhysicsStyle selectedStyle = (JellyMesh.PhysicsStyle)m_PhysicsStyle.enumValueIndex;

		if(m_PhysicsStyle.intValue != (int)JellyMesh.PhysicsStyle.Free)
		{
			EditorGUILayout.PropertyField(m_AttachNeighbors, new GUIContent("Attach Neighbors"));
		}

		int initialStyle = m_PhysicsStyle.enumValueIndex;
		EditorGUILayout.PropertyField(m_PhysicsStyle, new GUIContent("Body Configuration"));

		if(m_PhysicsStyle.enumValueIndex != initialStyle && (JellyMesh.PhysicsStyle)m_PhysicsStyle.enumValueIndex == JellyMesh.PhysicsStyle.Free)
		{
			m_PhysicsStyle.enumValueIndex = initialStyle;
			Debug.LogError("Please use the 'Copy Configuration to Free Mode' button to switch the Jelly Mesh to Free Mode");
		}
		
		switch(selectedStyle)
		{
		case JellyMesh.PhysicsStyle.Circle:
			EditorGUILayout.IntSlider(m_CircleRadiusPoints, 8, 128, new GUIContent("Num Colliders") );
			break;
		case JellyMesh.PhysicsStyle.Sphere:
			EditorGUILayout.IntSlider(m_CircleRadiusPoints, 4, 128, new GUIContent("Collider Density") );
			break;		
		case JellyMesh.PhysicsStyle.Rectangle:
			break;
		case JellyMesh.PhysicsStyle.Triangle:
			break;
		case JellyMesh.PhysicsStyle.Free:
			for(int i = 0; i < targetObject.m_FreeModeBodyPositions.Count; i++)
			{
				EditorGUILayout.BeginHorizontal();
				Vector3 point = targetObject.m_FreeModeBodyPositions[i];
				float radius = targetObject.m_FreeModeBodyRadii[i];
				bool kinematic = targetObject.m_FreeModeBodyKinematic[i];

				Vector4 result = Vector4.zero;

				if(i == 0) 
				{
					result = EditorGUILayout.Vector3Field("Centre", new Vector3(point.x, point.y, point.z), GUILayout.MaxWidth(1500f));
				}
				else
				{
					result = EditorGUILayout.Vector4Field("Body " + i .ToString(), new Vector4(point.x, point.y, point.z, radius), GUILayout.MaxWidth(1500f));
				}

				Vector3 positionResult = new Vector3(result.x, result.y, result.z);
				bool kinematicResult = false;

				if(point != positionResult)
				{
					Undo.RecordObject(targetObject, "Adjusted Jelly Mesh Body Position");
					targetObject.m_FreeModeBodyPositions[i] = positionResult;
					SceneView.RepaintAll();
				}

				if(result.w != radius)
				{
					Undo.RecordObject(targetObject, "Adjusted Jelly Mesh Body Radius");
					targetObject.m_FreeModeBodyRadii[i] = result.w;
					SceneView.RepaintAll();
				}

				using (new JellyMeshFixedWidthLabel("Kinematic"))
				{
					kinematicResult = EditorGUILayout.Toggle(kinematic);
				}

				if(kinematicResult != kinematic)
				{
					Undo.RecordObject(targetObject, "Adjusted Jelly Mesh Body Kinematic Flag");
					targetObject.m_FreeModeBodyKinematic[i] = kinematicResult;
					SceneView.RepaintAll();
				}

				if(GUILayout.Button(new GUIContent("Delete", "delete this point"), EditorStyles.miniButtonRight, GUILayout.MaxWidth(100f)))
				{
					if(i == 0)
					{
						Debug.LogWarning("Cannot remove central Jelly Mesh control point!");
					}
					else
					{
						Undo.RecordObject(targetObject, "Deleted Jelly Mesh Body");
						targetObject.m_FreeModeBodyPositions.RemoveAt(i);
						targetObject.m_FreeModeBodyRadii.RemoveAt(i);
						SceneView.RepaintAll();
						i--;
					}
				}

				EditorGUILayout.EndHorizontal();
			}

			GUILayout.Space(5);

			if(GUILayout.Button(new GUIContent("Add New Body", "Add new body"), EditorStyles.miniButtonLeft))
			{				
				Undo.RecordObject(targetObject, "Added New Jelly Mesh Body");
				targetObject.m_FreeModeBodyPositions.Add(new Vector2(0.0f, -1.0f));
				targetObject.m_FreeModeBodyRadii.Add(1.0f);
				targetObject.m_FreeModeBodyKinematic.Add(false);
				SceneView.RepaintAll();
			}

			break;
		}

		if(selectedStyle != JellyMesh.PhysicsStyle.Free)
		{
			EditorGUILayout.Slider(m_ColliderRadius, 0.001f, 0.25f, new GUIContent("Collider Radius") );
			EditorGUILayout.PropertyField(m_SoftBodyScale, new GUIContent("Collider Scale"));
			EditorGUILayout.PropertyField(m_SoftBodyOffset, new GUIContent("Collider Offset"));
			EditorGUILayout.PropertyField(m_SoftBodyRotation, new GUIContent("Collider Rotation"));
			EditorGUILayout.PropertyField(m_SoftBodyPivotOffset, new GUIContent("Pivot Offset"));

			GUILayout.Space(5);

			if(GUILayout.Button(new GUIContent("Copy Configuration To Free Mode", "Copy this configuration to the free mode layout"), EditorStyles.miniButtonRight))
			{
				Undo.RecordObject(target, "Converted Jelly Mesh to Free Mode");
				targetObject.OnCopyToFreeModeSelected();
				SceneView.RepaintAll();
			}
		}

		GUI.enabled = true;

		EditorGUILayout.PropertyField(m_LockRotationX, new GUIContent("Freeze Rotation X"));
		EditorGUILayout.PropertyField(m_LockRotationY, new GUIContent("Freeze Rotation Y"));
		EditorGUILayout.PropertyField(m_LockRotationZ, new GUIContent("Freeze Rotation Z"));

        if(!m_PhysicsMode.boolValue)
        {
            EditorGUILayout.PropertyField(m_LockPositionX, new GUIContent("Freeze Position X"));
            EditorGUILayout.PropertyField(m_LockPositionY, new GUIContent("Freeze Position Y"));
            EditorGUILayout.PropertyField(m_LockPositionZ, new GUIContent("Freeze Position Z"));
        }

		if(selectedStyle != JellyMesh.PhysicsStyle.Free)
		{
			EditorGUILayout.PropertyField(m_CentralBodyKinematic, new GUIContent("Kinematic Central Body"));
		}

		GUILayout.Space(15);

		if(m_PhysicsMode.boolValue)
		{
			EditorGUILayout.PropertyField(m_PhysicsMaterial2D, new GUIContent("Physics Material"));
			m_GravityScale.floatValue = EditorGUILayout.FloatField("Gravity Scale", m_GravityScale.floatValue);
            EditorGUILayout.PropertyField(m_InterpolationMode2D, new GUIContent("Interpolate"));
            EditorGUILayout.PropertyField(m_CollisionDetectionMode2D, new GUIContent("Collision Detection"));
		}
		else
		{
			EditorGUILayout.PropertyField(m_PhysicsMaterial, new GUIContent("Physics Material"));
            EditorGUILayout.PropertyField(m_UseGravity, new GUIContent("Use Gravity"));
            EditorGUILayout.PropertyField(m_InterpolationMode, new GUIContent("Interpolate"));
            EditorGUILayout.PropertyField(m_CollisionDetectionMode, new GUIContent("Collision Detection"));
		}

        EditorGUILayout.PropertyField(m_CollideConnected, new GUIContent("Collide Connected"));
        m_Drag.floatValue = EditorGUILayout.FloatField("Drag", m_Drag.floatValue);
        m_AngularDrag.floatValue = EditorGUILayout.FloatField("Angular Drag", m_AngularDrag.floatValue);

		m_Stiffness.floatValue = EditorGUILayout.FloatField("Spring Stiffness", m_Stiffness.floatValue);
		m_DampingRatio.floatValue = EditorGUILayout.FloatField("Spring Damping", m_DampingRatio.floatValue);
		EditorGUILayout.PropertyField(m_MassStyle, new GUIContent("Mass Type"));
		m_Mass.floatValue = EditorGUILayout.FloatField("Mass", m_Mass.floatValue);

		GUILayout.Space(15);

		if(Application.isPlaying)
		{
			GUI.enabled = false;
		}

		EditorGUILayout.Slider(m_DistanceExponent, 1.0f, 4.0f, new GUIContent("Ctrl Point Influence") );

		GUI.enabled = true;

		GUILayout.Space(15);

		m_NumAttachPoints.intValue = EditorGUILayout.IntField("Num Attach Points", m_NumAttachPoints.intValue);

		for(int loop = 0; loop < targetObject.m_AttachPoints.Length; loop++)
		{
			targetObject.m_AttachPoints[loop] = (Transform)EditorGUILayout.ObjectField("Attach Point", targetObject.m_AttachPoints[loop], typeof(Transform), true, null);
		}
	}

	public override void OnInspectorGUI() 
	{
		GUILayout.Space(5);
		StoreInitialValues();
		serializedObject.Update();
		DisplayInspectorGUI();
		CheckForObjectChanges();
	}	

	void OnSceneGUI()
	{
		UpdateHandles();
	}

	protected void UpdateHandles()
	{
		if(!Application.isPlaying)
		{
			JellyMesh jellyMesh = (JellyMesh)target;
			
			if(jellyMesh.m_Style == JellyMesh.PhysicsStyle.Free)
			{
				if(jellyMesh.m_FreeModeBodyPositions != null)
				{
					Transform transform = jellyMesh.transform;
					
					for(int i = 0; i < jellyMesh.m_FreeModeBodyPositions.Count; i++)
					{
						Handles.color = jellyMesh.m_FreeModeBodyKinematic[i]? Color.red : Color.green;
						
						Vector3 point = transform.TransformPoint(jellyMesh.m_FreeModeBodyPositions[i]);
						Vector3 newPosition = transform.InverseTransformPoint(Handles.FreeMoveHandle(point, Quaternion.identity, HandleUtility.GetHandleSize(point) * 0.05f, Vector3.zero, Handles.DotHandleCap));

						if(jellyMesh.m_FreeModeBodyPositions[i] != newPosition)
						{
							Undo.RecordObject(target, "Adjusted Jelly Mesh Body Position");
							jellyMesh.m_FreeModeBodyPositions[i] = newPosition;
						}

						if(i > 0)
						{
							float newRadius = Handles.RadiusHandle(Quaternion.identity, point, jellyMesh.m_FreeModeBodyRadii[i]);

							if(jellyMesh.m_FreeModeBodyRadii[i] != newRadius)
							{
								Undo.RecordObject(target, "Adjusted Jelly Mesh Body Radius");
								jellyMesh.m_FreeModeBodyRadii[i] = newRadius;
							}
						}
					}
				}
			}
		}
	}
}

public class JellyMeshFixedWidthLabel : IDisposable
{
	//helper class to clear and restore indentation
	private readonly JellyMeshZeroIndent indentReset;

	public JellyMeshFixedWidthLabel(GUIContent label)
	{
		//Create a horizontal group
		EditorGUILayout.BeginHorizontal();
		//Display the label:
		EditorGUILayout.LabelField(label,
		//Fix its width:
		GUILayout.Width(GUI.skin.label.CalcSize(label).x +
		//Correct for previous indentation: (9 pixels per level)
		9 * EditorGUI.indentLevel));
		//Set following indentation to zero:
		indentReset = new JellyMeshZeroIndent();
	}
	
	//alternative constructor, if we don't want to deal with GUIContents
	public JellyMeshFixedWidthLabel(string label) : this(new GUIContent(label))
	{
	}

	public void Dispose()
	{
		//restore indentation state:
		indentReset.Dispose();
		//finish horizontal group:
		EditorGUILayout.EndHorizontal();
	}
}

class JellyMeshZeroIndent : IDisposable //helper class to clear indentation
{
	private readonly int originalIndent;//the original indentation value before we change the GUI state
	public JellyMeshZeroIndent()
	{
		originalIndent = EditorGUI.indentLevel;//save original indentation
		EditorGUI.indentLevel = 0;//clear indentation
	}
	
	public void Dispose()
	{
		EditorGUI.indentLevel = originalIndent;//restore original indentation
	}
}
