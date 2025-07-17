using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MeshesDeformation : MonoBehaviour
{
    [SerializeField] private GameObject deformableObjects;
    [SerializeField] private GameObject[] deformableObjectsList;
    [SerializeField] private MeshFilter[] meshFilters = default;
    [SerializeField] private MeshCollider[] colliders = default;
    [SerializeField] private float[] detachHealth = default;
    [SerializeField] private float impactDamage = 1f;
    [SerializeField] private float deformationRadius = 0.5f;
    [SerializeField] private float maxDeformation = 0.5f;
    [SerializeField] private float minVelocity = 2f;
    private float delayTimeDeform = 0.1f;
    private float minVertsDistanceToRestore = 0.002f;
    private float vertsRestoreSpeed = 2f;
    private Vector3[][] originalVertices;
    private float nextTimeDeform = 0f;
    private bool isRepairing = false;
    private bool isRepaired = false;
    public float maxHealth = 50f;


    private void Start()
    {
        deformableObjectsList = new GameObject[deformableObjects.transform.childCount];
        meshFilters = new MeshFilter[deformableObjects.transform.childCount];
        colliders = new MeshCollider[deformableObjects.transform.childCount];
        detachHealth = new float[colliders.Length];

        for (int i = 0; i < deformableObjects.transform.childCount; i++)
        {
            deformableObjectsList[i] = deformableObjects.transform.GetChild(i).gameObject;
            meshFilters[i] = deformableObjectsList[i].GetComponent<MeshFilter>();
            colliders[i] = deformableObjectsList[i].GetComponent<MeshCollider>();
            detachHealth[i] = maxHealth;
        }
        //meshFilters = deformableObjects.GetComponentsInChildren<MeshFilter>();
        //colliders = deformableObjects.GetComponentsInChildren<MeshCollider>();

        if (deformableObjects != null)
        {

        }

        originalVertices = new Vector3[meshFilters.Length][];

        for (int i = 0; i < meshFilters.Length; i++)
        {
            originalVertices[i] = meshFilters[i].mesh.vertices;
            meshFilters[i].mesh.MarkDynamic();
        }
    }

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.R))
        {
            if (!isRepairing)
            {
                isRepairing = true;
            }
        }

        RestoreMesh();
    }

    private void DeformationMesh(Mesh mesh, Transform localTransform, Vector3 contactPoint, Vector3 contactVelocity, Vector3 contactNormal, int i)
    {
        bool hasDeformated = false;

        Vector3 localContactPoint = localTransform.InverseTransformPoint(contactPoint);
        Vector3 localContactForce = localTransform.InverseTransformDirection(contactVelocity);
        Vector3[] vertices = mesh.vertices;

        for (int j = 0; j < vertices.Length; j++)
        {
            float distance = (localContactPoint - vertices[j]).magnitude;

            if (distance <= deformationRadius)
            {
                vertices[j] += localContactForce * (deformationRadius - distance) * impactDamage;
                Vector3 deformation = vertices[j] - originalVertices[i][j];

                if (deformation.magnitude > maxDeformation)
                {
                    vertices[j] = originalVertices[i][j] + deformation.normalized * maxDeformation;
                }

                hasDeformated = true;
            }
        }

        if (hasDeformated)
        {
            mesh.vertices = vertices;
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();

            if (colliders.Length > 0)
            {
                if (colliders[i] != null)
                {
                    colliders[i].sharedMesh = mesh;
                }
            }

            UpdateCarPartDamage(contactNormal, contactVelocity, i);
        }
    }

    private void UpdateCarPartDamage(Vector3 contactNormal, Vector3 contactVelocity, int i)
    {
        Vector3 impactVelocity = contactVelocity / 0.02f;

        float dotMultiplier = Vector3.Dot(transform.up, contactNormal);

        if (dotMultiplier > 0.75f)
        {
            dotMultiplier = 1;
        }

        // Subtracting a minimum threshold can avoid tiny scratches at negligible speeds.
        //float magnitude = Mathf.Max(0f, impactVelocity.magnitude - 5);
        float magnitude = impactVelocity.magnitude;

        // Using sqrMagnitude can feel good here,
        // making light taps less damaging and high-speed strikes devastating.

        float damage = magnitude * (1 - Mathf.Max(dotMultiplier, 0)) * 1;
        //float damage = magnitude ;
        detachHealth[i] = detachHealth[i] - damage;
        if (detachHealth[i] < 0.66f * maxHealth)
        {
            Destroy(deformableObjectsList[i].GetComponent<FixedJoint>());

        }
        if (detachHealth[i] < 0.33f * maxHealth)
        {
            if (deformableObjectsList[i].GetComponent<HingeJoint>())
            {
                deformableObjectsList[i].GetComponent<HingeJoint>().breakForce = 50000;
                deformableObjectsList[i].GetComponent<HingeJoint>().breakForce = 10000f;
            }


        }
        if (detachHealth[i] < 0)
        {
            Destroy(deformableObjectsList[i].GetComponent<HingeJoint>());
            Destroy(deformableObjectsList[i].GetComponent<FixedJoint>());
            detachHealth[i] = 0;
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (Time.time > nextTimeDeform)
        {
            if (collision.relativeVelocity.magnitude > minVelocity)
            {
                isRepaired = false;

                Vector3 contactPoint = collision.contacts[0].point;
                Vector3 contactNormal = collision.contacts[0].normal;
                Vector3 contactVelocity = collision.relativeVelocity * 0.02f;

                for (int i = 0; i < meshFilters.Length; i++)
                {
                    if (meshFilters[i] != null)
                    {
                        DeformationMesh(meshFilters[i].mesh, meshFilters[i].transform, contactPoint, contactVelocity, contactNormal, i);
                    }
                }

                nextTimeDeform = Time.time + delayTimeDeform;
            }
        }
    }

    internal void overBoostDetachment()
    {
        for (int i = 0; i < deformableObjectsList.Length; i++)
        {
            HingeJoint[] hingeJoints = deformableObjectsList[i].GetComponents<HingeJoint>();
            FixedJoint[] fixedJoints = deformableObjectsList[i].GetComponents<FixedJoint>();

            for (int j = 0; j < fixedJoints.Length; j++)
            {
                fixedJoints[j].breakForce = Random.Range(5000, 10000);
                fixedJoints[j].breakTorque = fixedJoints[j].breakForce * 1.1f;
            }
            for (int j = 0; j < hingeJoints.Length; j++)
            {
                hingeJoints[j].breakForce = Random.Range(5000, 10000); ;
                hingeJoints[j].breakTorque = hingeJoints[j].breakForce * 1.1f;
            }
        }
    }


    private void RestoreMesh()
    {
        if (!isRepaired && isRepairing)
        {
            isRepaired = true;

            for (int i = 0; i < meshFilters.Length; i++)
            {
                Mesh mesh = meshFilters[i].mesh;
                Vector3[] vertices = mesh.vertices;
                Vector3[] origVerts = originalVertices[i];

                for (int j = 0; j < vertices.Length; j++)
                {
                    vertices[j] += (origVerts[j] - vertices[j]) * Time.deltaTime * vertsRestoreSpeed;

                    if ((origVerts[j] - vertices[j]).magnitude > minVertsDistanceToRestore)
                    {
                        isRepaired = false;
                    }
                }

                mesh.vertices = vertices;
                mesh.RecalculateNormals();
                mesh.RecalculateBounds();

                if (colliders[i] != null)
                {
                    colliders[i].sharedMesh = mesh;
                }
            }

            if (isRepaired)
            {
                isRepairing = false;

                for (int i = 0; i < meshFilters.Length; i++)
                {
                    if (colliders[i] != null)
                    {
                        colliders[i].sharedMesh = meshFilters[i].mesh;
                    }
                }
            }
        }
    }

    public Renderer getMeshMaterial()
    {
        return deformableObjectsList[0].GetComponent<Renderer>();
    }
}
