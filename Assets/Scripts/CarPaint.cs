using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CarPaint : MonoBehaviour
{
    public GameObject go;
    public Color color;
    Material material;
    Transform carComponents;

    public bool allowActiveColoring = false;
    // Start is called before the first frame update
    void Start()
    {
        go = gameObject;
        carComponents = transform.Find("DeformableObjects");
        material = Resources.Load("Materials/Car/CarPaint", typeof(Material)) as Material;
        material.SetColor("_CarColor", color);
        bool init = true;

        foreach (Transform child in carComponents)
        {
            MeshRenderer childMR = child.GetComponent<MeshRenderer>();
            if (init)
            {
                childMR.material = material;
                childMR.material.SetColor("_CarColor", color);
                material = childMR.material;
                init = false;

            }
            else
            {
                            // Access properties or call methods on the 'child' transform
            
                childMR.sharedMaterial = material;
                // Example: child.gameObject.SetActive(false);
            
            }

        }
    }

}
