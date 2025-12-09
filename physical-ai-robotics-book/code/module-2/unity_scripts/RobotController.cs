using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class RobotController : MonoBehaviour
{
    public string topicName = "/cmd_vel";
    
    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<TwistMsg>(topicName, MoveRobot);
    }

    void MoveRobot(TwistMsg msg)
    {
        // Simple mapping of Twist linear.x to Unity forward movement
        float speed = (float)msg.linear.x;
        transform.Translate(Vector3.forward * speed * Time.deltaTime);
        
        // Mapping angular.z to rotation
        float rotation = (float)msg.angular.z;
        transform.Rotate(Vector3.up, -rotation * Mathf.Rad2Deg * Time.deltaTime);
    }
}
