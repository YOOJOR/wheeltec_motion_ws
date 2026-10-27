"""Generate the world, robot description and runtime configs from one YAML."""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from .model import robot_sdf, robot_urdf


def box(name, x, y, length, width, height, color, z=None, texture=None, collision=True):
    shape = f'<geometry><box><size>{length} {width} {height}</size></box></geometry>'
    material = f'<ambient>{color} 1</ambient><diffuse>{color} 1</diffuse>'
    if texture:
        material += f'<pbr><metal><albedo_map>{texture}</albedo_map><roughness>0.85</roughness><metalness>0</metalness></metal></pbr>'
    return f'<model name="{name}"><static>true</static><pose>{x} {y} {height/2 if z is None else z} 0 0 0</pose><link name="link">' + (f'<collision name="collision">{shape}</collision>' if collision else '') + f'<visual name="visual">{shape}<material>{material}</material></visual></link></model>'


def generate(config_path, output, motion_template):
    config_path = Path(config_path).resolve()
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    c = yaml.safe_load(config_path.read_text())
    s = c['scene']; w, d, t, h = (s[k] for k in ('width','depth','wall_thickness','wall_height'))
    assets = config_path.parent.parent / 'assets'
    textures = {'tile.png', 'plaster.png'} | {part['texture'] for item in s['furniture'] for part in item['parts'] if 'texture' in part}
    for name in textures:
        if not (assets/name).is_file():raise FileNotFoundError(assets/name)
    objects = [(-w/2, 0, t, d+t, h), (w/2, 0, t, d+t, h),
               (0, -d/2, w+t, t, h), (0, d/2, w+t, t, h)]
    geometry = box('floor', 0, 0, w+1, d+1, .1, '1 1 1', z=-.05,
                   texture=(assets/'tile.png').as_uri())
    for i, obj in enumerate(objects):
        geometry += box(f'wall_{i}', *obj, '1 1 1', texture=(assets/'plaster.png').as_uri())
    for item in s['furniture']:
        for i, part in enumerate(item['parts']):
            x,y,l,b,height,z = part['box']
            solid = part.get('collision', True)
            geometry += box(f"{item['name']}_{i}", x,y,l,b,height,part['color'],z=z,
                            texture=(assets/part['texture']).as_uri() if 'texture' in part else None,
                            collision=solid)
            if solid:
                objects.append((x,y,l,b,height))
    robot = ET.fromstring(robot_sdf(c))
    model = robot.find('model') if robot.tag == 'sdf' else robot
    x,y,z,yaw = c['spawn']
    ET.SubElement(model, 'pose').text = f'{x} {y} {z} 0 0 {yaw}'
    world = f'''<sdf version="1.8"><world name="{c['world']}">
<physics name="physics" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics>
<plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
<plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
<plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
<plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
<gravity>0 0 -9.81</gravity><light type="directional" name="sun"><pose>0 0 5 0 0 0</pose><diffuse>.9 .9 .9 1</diffuse><direction>-.3 .1 -1</direction></light>
{geometry}{ET.tostring(model, encoding='unicode')}</world></sdf>'''
    (output/'room.sdf').write_text(world)
    (output/'robot.urdf').write_text(robot_urdf(c))
    resolution = s['map_resolution']; nx, ny = round(w/resolution), round(d/resolution)
    xx = -w/2 + (np.arange(nx)+.5)*resolution
    yy = d/2 - (np.arange(ny)+.5)*resolution
    occupancy = np.full((ny,nx), 254, np.uint8)
    for x,y,l,b,_ in objects:
        occupancy[(abs(yy[:,None]-y)<=b/2) & (abs(xx[None,:]-x)<=l/2)] = 0
    (output/'map.pgm').write_bytes(f'P5\n{nx} {ny}\n255\n'.encode()+occupancy.tobytes())
    (output/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm', resolution=resolution,
        origin=[-w/2,-d/2,0.0],negate=0,occupied_thresh=.65,free_thresh=.196)))
    (output/'navigation.yaml').write_text(yaml.safe_dump(c['navigation']))
    motion = yaml.safe_load(Path(motion_template).read_text())
    params = motion['wheeltec_motion_controller']['ros__parameters']
    params.update(control_enabled=True,extrinsics_calibrated=True,
        max_linear_speed=c['motor']['max_linear_speed'],
        max_angular_speed=c['motor']['max_angular_speed'],
        body_from_base_translation=c['adapter']['body_from_base_translation'],
        body_from_base_rpy=c['adapter']['body_from_base_rpy'])
    (output/'motion.yaml').write_text(yaml.safe_dump(motion))
    optical = [[0.,0.,1.,c['camera']['position'][0]],[-1.,0.,0.,c['camera']['position'][1]],
               [0.,-1.,0.,c['camera']['position'][2]],[0.,0.,0.,1.]]
    agent = dict(continuous_exploration=True,base_frame='base_link',base_height=c['model']['base_height'],odom_topic='/Odometry',
        motion_config=str(output/'motion.yaml'),camera=dict(base_from_camera=optical,confirmed=True,source='gazebo_model'))
    (output/'agent.yaml').write_text(yaml.safe_dump(agent))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config',required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--motion-template',default='/sim/ws/src/wheeltec_motion_control/config/motion_control.yaml')
    args = parser.parse_args()
    generate(args.config,args.output,args.motion_template)


if __name__ == '__main__':
    main()
