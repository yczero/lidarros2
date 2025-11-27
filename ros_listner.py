# ros_listener.py
import roslibpy

# ROS 브리지 서버 연결

# client = roslibpy.Ros(host='192.168.56.101', port=9090)
client = roslibpy.Ros(host='localhost', port=9090)
client.run()
print('Connected to ROS bridge server.')

# /scan 토픽 구독
scan_topic = roslibpy.Topic(client, '/scan', 'sensor_msgs/LaserScan')

def scan_callback(message):
    ranges = message['ranges']

    # 간단한 액션 결정 예시
    front = min(ranges[0:10] + ranges[-10:])  # 정면 최소 거리
    left = min(ranges[80:100])                # 왼쪽 최소 거리
    right = min(ranges[260:280])              # 오른쪽 최소 거리

    action = ''
    if front < 0.5:
        if left > right:
            action = 'Turn Left'
        else:
            action = 'Turn Right'
    else:
        action = 'Go Forward'

    print(f"Action: {action} (Front:{front:.2f}, Left:{left:.2f}, Right:{right:.2f})")

scan_topic.subscribe(scan_callback)

try:
    while True:
        pass
except KeyboardInterrupt:
    scan_topic.unsubscribe()
    client.terminate()
    print("Disconnected from ROS bridge server.")