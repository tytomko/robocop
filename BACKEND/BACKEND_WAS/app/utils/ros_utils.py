import roslibpy
import asyncio

async def get_topic_list(ros_host='192.168.100.104', ros_port=9090):
    try:
        # ROS Bridge 연결
        client = roslibpy.Ros(host=ros_host, port=ros_port)
        client.run()
        
        # ROS2 토픽 리스트를 가져오기 위한 서비스 호출
        service = roslibpy.Service(client, '/rosapi/topics', 'rosapi/Topics')
        result = service.call(roslibpy.ServiceRequest())
        
        print("\n=== Available ROS Topics ===")
        for topic, topic_type in zip(result['topics'], result['types']):
            print(f"Topic: {topic}")
            print(f"Type: {topic_type}")
            print("---")
        
        client.terminate()
        
    except Exception as e:
        print(f"Error getting topic list: {e}")
        import traceback
        traceback.print_exc()

# 사용 예시
if __name__ == "__main__":
    asyncio.run(get_topic_list()) 