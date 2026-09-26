"""Passive probe; raw subscriptions avoid decoding large point arrays."""
import argparse
import time
import struct
import statistics

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--seconds', type=float, default=15.0, help='sampling duration (0 < seconds <= 300)')
options = parser.parse_args()
if not 0 < options.seconds <= 300:
    parser.error('--seconds must be in (0, 300]')
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from livox_ros_driver2.msg import CustomMsg
from rclpy.serialization import deserialize_message

rclpy.init()
node = rclpy.create_node('wheeltec_delay_probe')
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                 durability=DurabilityPolicy.VOLATILE)
window = {name: [] for name in ('imu', 'lidar', 'odom')}
all_rows = {name: [] for name in window}
first_lidar = None
subscriptions = []

def callback(name):
    def receive(data):
        global first_lidar
        received = time.time_ns() * 1e-9
        endian = '<' if data[1] == 1 else '>'
        sec, ns = struct.unpack_from(endian+'iI', data, 4)
        stamp = sec + ns * 1e-9
        row = [received-stamp]
        if name == 'lidar':
            string_len = struct.unpack_from(endian+'I', data, 12)[0]
            pos = 16 + string_len
            pos = 4 + ((pos-4+7)//8)*8
            timebase, point_num = struct.unpack_from(endian+'QI', data, pos)
            points_start = pos + 20
            count = struct.unpack_from(endian+'I', data, pos+16)[0]
            if count and points_start + (count-1)*20 + 19 <= len(data):
                first = struct.unpack_from(endian+'I', data, points_start)[0]
                last = struct.unpack_from(endian+'I', data, points_start+(count-1)*20)[0]
                row += [received-(timebase+last)*1e-9, (last-first)*1e-9, point_num]
            if first_lidar is None:
                first_lidar = bytes(data)
        window[name].append(row)
        all_rows[name].append(row)
    return receive

for name, topic, cls in [('imu','/livox/imu',Imu), ('lidar','/livox/lidar',CustomMsg), ('odom','/Odometry',Odometry)]:
    subscriptions.append(node.create_subscription(cls, topic, callback(name), qos, raw=True))
started = time.monotonic()
last_report = started
while time.monotonic() - started < options.seconds:
    rclpy.spin_once(node, timeout_sec=0.05)
    now = time.monotonic()
    if now-last_report >= 1:
        parts=[]
        for name, rows in window.items():
            if rows:
                ages=[r[0] for r in rows]
                extra=''
                lidar_rows = [r for r in rows if len(r) == 4]
                if name=='lidar' and lidar_rows:
                    extra=f' end_age={statistics.mean(r[1] for r in lidar_rows):.3f}s span={statistics.mean(r[2] for r in lidar_rows):.3f}s points={lidar_rows[-1][3]}'
                parts.append(f'{name}: n={len(rows)} age={statistics.mean(ages):.3f}s [{min(ages):.3f},{max(ages):.3f}]'+extra)
            else:
                parts.append(name+': no data')
            rows.clear()
        print(' | '.join(parts),flush=True)
        last_report=now
for name, rows in all_rows.items():
    if rows:
        print(f'TOTAL {name}: n={len(rows)}, mean_age={statistics.mean(r[0] for r in rows):.6f}s',flush=True)
if first_lidar:
    msg=deserialize_message(first_lidar,CustomMsg)
    endian='<' if first_lidar[1]==1 else '>'
    string_len=struct.unpack_from(endian+'I',first_lidar,12)[0]
    pos=4+((16+string_len-4+7)//8)*8
    timebase,num=struct.unpack_from(endian+'QI',first_lidar,pos)
    count=struct.unpack_from(endian+'I',first_lidar,pos+16)[0]
    assert timebase==msg.timebase and num==msg.point_num and count==len(msg.points)
    if count:
        assert struct.unpack_from(endian+'I',first_lidar,pos+20)[0]==msg.points[0].offset_time
        assert struct.unpack_from(endian+'I',first_lidar,pos+20+(count-1)*20)[0]==msg.points[-1].offset_time
    print('PASS: raw lidar parser matches ROS deserialization for first captured frame.',flush=True)
node.destroy_node()
rclpy.shutdown()
