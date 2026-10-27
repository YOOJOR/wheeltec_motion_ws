"""Physical mecanum model. +X forward, +Y left, +Z up; base at axle XY centre.

Wheel order LF, LB, RB, RF; all driven joint axes are +Y. Each hub carries
8 freely rotating 45 degree capsule rollers in X configuration. Camera link
uses +X forward/+Z up, d455_color_optical_frame uses +Z forward/+X right/+Y down.
SDF model pose is supplied by scene.py; base_link sits 0.026 m above ground.
"""
from math import acos, atan2, cos, pi, sin, sqrt
from xml.sax.saxutils import escape


def _parameters(config):
    m = config.get('model', {})
    c = config.get('camera', {})
    return dict(length=float(m.get('chassis_length', .36)),
                width=float(m.get('chassis_width', .28)),
                height=float(m.get('chassis_height', .12)),
                mass=float(m.get('mass', 3.0)),
                base=float(m.get('base_height', .026)),
                radius=float(m.get('wheel_radius', .05)),
                wheel_width=float(m.get('wheel_width', .04)),
                axle=float(m.get('wheelbase', .313)),
                track=float(m.get('track', .355)),
                wheel_mass=float(m.get('wheel_mass', .224)), camera=c)


def _inertia(mass, x, y, z):
    return (f'<inertial><mass>{mass}</mass><inertia>'
            f'<ixx>{mass*(y*y+z*z)/12}</ixx><iyy>{mass*(x*x+z*z)/12}</iyy>'
            f'<izz>{mass*(x*x+y*y)/12}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>'
            '</inertia></inertial>')


def _surface():
    return ('<surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>'
            '<contact><ode><kp>100000</kp><kd>10</kd><max_vel>0.1</max_vel>'
            '<min_depth>0.0001</min_depth></ode></contact></surface>')


def _visual(name, geometry, color, pose='0 0 0 0 0 0'):
    return (f'<visual name="{name}_visual"><pose>{pose}</pose><geometry>{geometry}</geometry>'
            f'<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>')


def robot_sdf(config: dict) -> str:
    """Return one <model> element for Gazebo Fortress (SDFormat 1.9)."""
    p = _parameters(config)
    r, w, base = p['radius'], p['wheel_width'], p['base']
    # Keep the chassis above the wheel ground contact; its centre is at 0.12 m.
    chassis_z = r + p['height']/2 + .01 - base
    box = f"<box><size>{p['length']} {p['width']} {p['height']}</size></box>"
    pieces = ['<model name="conavbot"><self_collide>false</self_collide>',
              '<link name="base_link">',
              f'<inertial><pose>0 0 {chassis_z} 0 0 0</pose><mass>{p["mass"]}</mass>'
              f'<inertia><ixx>{p["mass"]*(p["width"]**2+p["height"]**2)/12}</ixx>'
              f'<iyy>{p["mass"]*(p["length"]**2+p["height"]**2)/12}</iyy>'
              f'<izz>{p["mass"]*(p["length"]**2+p["width"]**2)/12}</izz>'
              '<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>',
              f'<collision name="chassis"><pose>0 0 {chassis_z} 0 0 0</pose><geometry>{box}</geometry></collision>',
              _visual('chassis', box, '.12 .25 .40 1', f'0 0 {chassis_z} 0 0 0'), '</link>']
    # At the bottom contact LF/RB roller axis is proportional to (+X,-Y);
    # LB/RF is (+X,+Y), giving the standard X inverse wheel kinematics.
    wheels = [('lf', 1, 1, 1), ('lb', -1, 1, -1),
              ('rb', -1, -1, 1), ('rf', 1, -1, -1)]
    roller_radius = .20*r
    roller_length = .70*w
    roller_centre = r-roller_radius
    roller_mass = p['wheel_mass']*.025
    hub_mass = p['wheel_mass']-8*roller_mass
    for name, fore, side, handed in wheels:
        x, y, z = fore*p['axle']/2, side*p['track']/2, r-base
        hub_geometry = f'<cylinder><radius>{.65*r}</radius><length>{.75*w}</length></cylinder>'
        pieces += [f'<link name="{name}_wheel"><pose>{x} {y} {z} 0 0 0</pose>',
                   _inertia(hub_mass, 1.3*r, w, 1.3*r),
                   _visual('hub', hub_geometry, '.10 .10 .10 1', f'0 0 0 {pi/2} 0 0'), '</link>',
                   f'<joint name="{name}_wheel_joint" type="revolute"><parent>base_link</parent>'
                   f'<child>{name}_wheel</child><axis><xyz>0 1 0</xyz>'
                   '<limit><lower>-1e16</lower><upper>1e16</upper></limit>'
                   '<dynamics><damping>0.001</damping></dynamics></axis></joint>']
        for i in range(8):
            theta = 2*pi*i/8
            # Z axis of capsule and passive joint follows tangent +/- wheel axis.
            axis = (cos(theta)/sqrt(2), handed/sqrt(2), -sin(theta)/sqrt(2))
            pitch = acos(axis[2])
            yaw = atan2(axis[1], axis[0])
            rx, rz = roller_centre*sin(theta), roller_centre*cos(theta)
            geom = f'<capsule><radius>{roller_radius}</radius><length>{roller_length}</length></capsule>'
            link = f'{name}_roller_{i}'
            pieces += [f'<link name="{link}"><pose relative_to="{name}_wheel">{rx} 0 {rz} 0 {pitch} {yaw}</pose>',
                       _inertia(roller_mass, 2*roller_radius, 2*roller_radius, roller_length+2*roller_radius),
                       f'<collision name="tread"><geometry>{geom}</geometry>{_surface()}</collision>',
                       _visual('tread', geom, '.24 .24 .24 1'), '</link>',
                       f'<joint name="{link}_joint" type="revolute"><parent>{name}_wheel</parent><child>{link}</child>'
                       '<axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit>'
                       '<dynamics><damping>0.00001</damping></dynamics></axis></joint>']
    c = p['camera']
    motor = config.get('motor', {})
    motor_defaults = [('command_delay', .20), ('kp', .08), ('ki', .10),
                      ('kd', 0), ('max_torque', .8), ('max_wheel_speed', 20),
                      ('torque_time_constant', .05), ('watchdog', .5),
                      ('control_rate', 100)]
    motor_xml = ''.join(f'<{key}>{float(motor.get(key, default))}</{key}>'
                        for key, default in motor_defaults)
    pos = c.get('position', [.15, 0, .30-base])
    camera_pose = ' '.join(str(float(v)) for v in pos) + ' 0 0 0'
    pieces += [f'<link name="camera_link"><pose>{camera_pose}</pose>',
               _inertia(.001, .02, .06, .02),
               '<sensor name="rgbd" type="rgbd_camera"><pose>0 0 0 0 0 0</pose>'
               '<topic>/sim/rgbd</topic><always_on>true</always_on>'
               f'<update_rate>{c.get("rate", 15)}</update_rate><camera>'
               f'<horizontal_fov>{c.get("hfov", 79*pi/180)}</horizontal_fov>'
               f'<image><width>{c.get("width", 640)}</width><height>{c.get("height", 480)}</height>'
               '<format>R8G8B8</format></image>'
               f'<clip><near>{c.get("near", .1)}</near><far>{c.get("far", 6)}</far></clip>'
               '<depth_camera><clip>'
               f'<near>{c.get("near", .1)}</near><far>{c.get("far", 6)}</far>'
               '</clip></depth_camera></camera></sensor></link>',
               '<joint name="camera_joint" type="fixed"><parent>base_link</parent><child>camera_link</child></joint>',
               '<plugin filename="libconavbot_motor.so" name="conavbot::MotorSystem">'
               f'{motor_xml}<wheel_radius>{r}</wheel_radius><axle>{p["axle"]}</axle><track>{p["track"]}</track></plugin>',
               '<plugin filename="ignition-gazebo-odometry-publisher-system" name="ignition::gazebo::systems::OdometryPublisher">'
               '<odom_frame>camera_init</odom_frame><robot_base_frame>base_link</robot_base_frame>'
               '<odom_publish_frequency>50</odom_publish_frequency><dimensions>3</dimensions>'
               '<odom_topic>/sim/odometry</odom_topic></plugin>', '</model>']
    return ''.join(pieces)


def robot_urdf(config: dict) -> str:
    """Return RViz geometry and base/camera/wheel TF; wheel frames are static.

    Gazebo owns articulated rollers. RViz shows cylindrical wheel envelopes;
    these fixed visual joints do not send commands or change simulator physics.
    """
    p = _parameters(config)
    r, w = p['radius'], p['wheel_width']
    z = r+p['height']/2+.01-p['base']
    pieces = ['<?xml version="1.0"?><robot name="conavbot">',
              f'<link name="base_link"><visual><origin xyz="0 0 {z}"/><geometry>'
              f'<box size="{p["length"]} {p["width"]} {p["height"]}"/>'
              '</geometry><material name="blue"><color rgba=".12 .25 .40 1"/></material></visual></link>']
    for name, fore, side in [('lf', 1, 1), ('lb', -1, 1), ('rb', -1, -1), ('rf', 1, -1)]:
        pieces += [f'<link name="{name}_wheel"><visual><origin rpy="{pi/2} 0 0"/>'
                   f'<geometry><cylinder radius="{r}" length="{w}"/></geometry>'
                   '<material name="black"><color rgba=".1 .1 .1 1"/></material></visual></link>',
                   f'<joint name="{name}_wheel_joint" type="fixed"><parent link="base_link"/>'
                   f'<child link="{name}_wheel"/><origin xyz="{fore*p["axle"]/2} {side*p["track"]/2} {r-p["base"]}"/></joint>']
    pos = ' '.join(str(float(v)) for v in p['camera'].get('position', [.15, 0, .30-p['base']]))
    pieces += ['<link name="camera_link"/><joint name="camera_joint" type="fixed">'
               f'<parent link="base_link"/><child link="camera_link"/><origin xyz="{escape(pos)}"/></joint>',
               '<link name="d455_color_optical_frame"/><joint name="camera_optical_joint" type="fixed">'
               '<parent link="camera_link"/><child link="d455_color_optical_frame"/>'
               f'<origin rpy="{-pi/2} 0 {-pi/2}"/></joint></robot>']
    return ''.join(pieces)
