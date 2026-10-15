"""Bounded mapping, initial observation turn and paper-style frontiers; no device I/O."""
from dataclasses import dataclass, asdict
import math
import cv2
import numpy as np
from scipy import ndimage
from wheeltec_rgbd.posed import rigid
from wheeltec_rgbd.frame_tree import ground_point


@dataclass(frozen=True)
class Config:
    base_height: float = 0.0
    exploration_policy: str = 'paper_frontier'
    initial_scan_angle: float = 2*math.pi
    initial_scan_timeout: float = 45.
    paper_frontier_min_cells: int = 20
    fallback_sample_attempts: int = 64
    resolution: float = .05
    size_m: float = 12.
    pixel_stride: int = 8
    obstacle_min_height: float = .08
    obstacle_max_height: float = 1.3
    floor_tolerance: float = .08
    footprint_radius: float = .25
    clearance: float = .05
    frontier_min_cells: int = 3
    goal_min_distance: float = .45
    goal_tolerance: float = .25
    goal_no_progress_sec: float = 10.
    goal_blacklist_sec: float = 15.
    turn_threshold: float = .35
    forward_speed: float = .10
    turn_speed: float = .25
    target_approach_distance: float = .6
    seed: int = 1

    def __post_init__(self):
        positive = ('resolution','size_m','obstacle_min_height','obstacle_max_height',
                    'floor_tolerance','footprint_radius','goal_min_distance','goal_tolerance',
                    'goal_no_progress_sec','goal_blacklist_sec','turn_threshold',
                    'forward_speed','turn_speed','target_approach_distance')
        if (not all(math.isfinite(getattr(self,k)) and getattr(self,k)>0 for k in positive)
                or self.exploration_policy not in ('paper_frontier','known_free')
                or not math.isfinite(self.initial_scan_angle) or not 0 <= self.initial_scan_angle <= 2*math.pi
                or not math.isfinite(self.initial_scan_timeout) or self.initial_scan_timeout <= 0
                or type(self.paper_frontier_min_cells) is not int or self.paper_frontier_min_cells < 1
                or type(self.fallback_sample_attempts) is not int or not 1 <= self.fallback_sample_attempts <= 1000
                or not math.isfinite(self.clearance) or self.clearance < 0
                or type(self.pixel_stride) is not int or self.pixel_stride < 1
                or type(self.frontier_min_cells) is not int or self.frontier_min_cells < 1
                or type(self.seed) is not int or self.seed < 0
                or not 20 <= round(self.size_m/self.resolution) <= 600
                or self.obstacle_min_height >= self.obstacle_max_height
                or self.goal_tolerance >= self.goal_min_distance
                or self.forward_speed > .15 or self.turn_speed > .35
                or not math.isclose(self.resolution,.05)
                or not math.isclose(self.goal_tolerance,.25)):
            raise ValueError('INVALID_EXPLORATION_CONFIG')


def unproject(depth, k, stride):
    depth, k = np.asarray(depth), np.asarray(k, float)
    if (depth.ndim != 2 or k.shape != (3,3) or not np.isfinite(k).all()
            or k[0,0] <= 0 or k[1,1] <= 0 or not np.allclose(k[2],[0,0,1])):
        raise ValueError('INVALID_RGBD_GEOMETRY')
    v, u = np.mgrid[0:depth.shape[0]:stride,0:depth.shape[1]:stride]
    z = depth[::stride,::stride]
    valid = np.isfinite(z) & (z>0)
    return np.column_stack(((u[valid]-k[0,2])*z[valid]/k[0,0],
                            (v[valid]-k[1,2])*z[valid]/k[1,1], z[valid]))


def line(a, b):
    """Grid samples including endpoints; integer rounding, no points beyond segment."""
    a, b = np.asarray(a), np.asarray(b)
    count = int(np.max(np.abs(b-a)))+1
    samples = np.rint(np.linspace(a,b,count)).astype(int)
    return samples[np.r_[True, np.any(np.diff(samples,axis=0),axis=1)]]


class Grid:
    def __init__(self, cfg, origin_xy):
        self.cfg = cfg
        self.n = round(cfg.size_m/cfg.resolution)
        self.origin = np.asarray(origin_xy, float)-self.n*cfg.resolution/2
        self.free = np.zeros((self.n,self.n),bool)
        # Point-projection layer from the paper, distinct from ray-carved free space.
        self.explored = np.zeros_like(self.free)
        self.obstacle = np.zeros_like(self.free)
        self.version = 0

    def cell(self, xy):
        col, row = np.floor((np.asarray(xy)-self.origin)/self.cfg.resolution).astype(int)
        if not (0 <= row < self.n and 0 <= col < self.n):
            raise ValueError('MAP_BOUNDARY')
        return int(row), int(col)

    def xy(self, cell):
        row, col = cell
        return self.origin+(np.array([col,row])+.5)*self.cfg.resolution

    def update(self, points, camera_xyz, base_xyz):
        start = self.cell(camera_xyz[:2])
        base = self.cell(base_xyz[:2])
        cells = np.floor((points[:,:2]-self.origin)/self.cfg.resolution).astype(int)
        inside = np.all((cells >= 0)&(cells < self.n),axis=1)
        self.explored[cells[inside,1],cells[inside,0]] = True
        relative_z = points[:,2]-base_xyz[2]
        obstacles = ((relative_z >= self.cfg.obstacle_min_height)
                     & (relative_z <= self.cfg.obstacle_max_height))
        floor = np.abs(relative_z) <= self.cfg.floor_tolerance
        rays = {}
        for point, is_obstacle, useful in zip(points, obstacles, obstacles|floor):
            if not useful:
                continue  # Overhead/invalid-height returns don't prove floor clearance.
            try: end = self.cell(point[:2])
            except ValueError: continue
            rays[end] = rays.get(end,False) or bool(is_obstacle)
        # Insert all obstacles before clearing rays, preventing same-frame occlusion leaks.
        for end, occupied in rays.items():
            if occupied: self.obstacle[end] = True
        if rays:
            ends = np.asarray(list(rays),int)
            start_array = np.asarray(start)
            steps = int(np.max(np.abs(ends-start_array)))+1
            paths = np.rint(start_array+(ends[:,None,:]-start_array)*
                            np.linspace(0,1,steps)[None,:,None]).astype(int)
            rows,cols = paths[...,0],paths[...,1]
            blocked = np.maximum.accumulate(self.obstacle[rows,cols],axis=1)
            self.free[rows[~blocked],cols[~blocked]] = True
        # Current body footprint is provisionally occupied by the robot, not unknown floor.
        radius = math.ceil(self.cfg.footprint_radius/self.cfg.resolution)
        rows, cols = np.ogrid[:self.n,:self.n]
        self.free |= (rows-base[0])**2+(cols-base[1])**2 <= radius**2
        self.free[self.obstacle] = False
        self.version += 1

    def reachable(self, base_cell, *, allow_unknown=False):
        radius = math.ceil((self.cfg.footprint_radius+self.cfg.clearance)/self.cfg.resolution)
        r,c = np.mgrid[-radius:radius+1,-radius:radius+1]
        inflated = ndimage.binary_dilation(self.obstacle, structure=r*r+c*c<=radius*radius)
        traversable = ~inflated if allow_unknown else self.free & ~inflated
        if not traversable[base_cell]: return np.zeros_like(traversable)
        labels,_ = ndimage.label(traversable)
        return labels == labels[base_cell]

    def paper_frontiers(self):
        """Global_Map_Proc.Frontier_Det geometry, adapted to ROS XY row/column.

        Locked Co-NavGPT2 utils/explored_map_utils.py: 5x5 close, largest contour,
        7x7 obstacle dilation, 8-connected groups, strict area threshold, up to 6.
        Goal selection is performed separately, replacing the VLM with seeded RNG.
        """
        closed = cv2.morphologyEx(self.explored.astype(np.uint8),cv2.MORPH_CLOSE,
                                 np.ones((5,5),np.uint8))
        contours,_ = cv2.findContours(closed,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
        edge = np.zeros_like(closed)
        if contours:
            cv2.drawContours(edge,[max(contours,key=cv2.contourArea)],-1,1,1)
        edge[:2,:] = edge[-2:,:] = 0
        edge[:,:2] = edge[:,-2:] = 0
        obstacles = cv2.dilate(self.obstacle.astype(np.uint8),np.ones((7,7),np.uint8))
        edge = (edge != 0)&(obstacles == 0)
        labels,count = ndimage.label(edge,structure=np.ones((3,3)))
        groups = []
        for i in range(1,count+1):
            cells = np.argwhere(labels == i)
            if len(cells) > self.cfg.paper_frontier_min_cells:
                groups.append((len(cells),tuple(cells.mean(axis=0).astype(int))))
        # Match upstream's stable ascending area order before its six-group cap.
        return [point for _,point in sorted(groups,key=lambda v:v[0])[:6]],edge

    def frontiers(self, reachable, base_cell):
        unknown = ~(self.free|self.obstacle)
        edge = reachable & ndimage.binary_dilation(unknown)
        edge[[0,-1],:] = False
        edge[:,[0,-1]] = False
        labels, count = ndimage.label(edge, structure=np.ones((3,3)))
        goals = []
        for i in range(1,count+1):
            cells = np.argwhere(labels == i)
            if len(cells) < self.cfg.frontier_min_cells: continue
            distances = np.linalg.norm((cells-np.array(base_cell))*self.cfg.resolution, axis=1)
            cells = cells[distances >= self.cfg.goal_min_distance]
            if not len(cells): continue
            center = cells.mean(axis=0)
            cell = cells[np.argmin(np.linalg.norm(cells-center,axis=1))]
            goals.append(tuple(int(v) for v in cell))
        return goals, edge

    def occupancy(self):
        data = np.full(self.free.shape,-1,np.int8)
        observed = self.explored if self.cfg.exploration_policy == 'paper_frontier' else self.free
        data[observed] = 0
        data[self.obstacle] = 100
        return data


@dataclass
class Decision:
    reason: str
    action: str = 'stop'
    linear_x: float = 0.
    angular_z: float = 0.
    goal_xy: object = None
    short_term_xy: object = None
    frontier_count: int = 0
    points: int = 0
    phase: str = 'EXPLORING'
    scan_progress_rad: float = 0.
    scan_target_rad: float = 0.
    goal_source: str = 'none'
    planning_allows_unknown: bool = False

    def as_dict(self): return asdict(self)


class Explorer:
    def __init__(self, cfg, fmm_class):
        self.cfg, self.fmm_class = cfg, fmm_class
        self.rng = np.random.default_rng(cfg.seed)
        self.reset()

    def reset(self):
        self.grid = None
        self.goal = None
        self.best_distance = math.inf
        self.progress_at = None
        self.blacklist = {}
        self.last_stamp = 0
        self.target_xy = None
        self.goal_kind = 'random_frontier'
        self.halted = None
        self.scan_started = None
        self.scan_last_yaw = None
        self.scan_progress = 0.
        self.scan_complete = self.cfg.initial_scan_angle == 0
        self.frontier_goals = []
        self.frontier_edge = None

    def halt(self, reason):
        self.grid = None
        self.goal = None
        self.blacklist.clear()
        self.target_xy = None
        self.frontier_goals = []
        self.frontier_edge = None
        self.halted = reason
        return Decision(reason,phase='HALTED')

    def initial_scan(self, base_matrix, monotonic):
        """Progress comes only from new paired poses, never elapsed callback count."""
        if self.scan_complete: return None
        if self.cfg.exploration_policy == 'paper_frontier':
            self.frontier_goals,self.frontier_edge = self.grid.paper_frontiers()
        yaw = math.atan2(base_matrix[1,0],base_matrix[0,0])
        if self.scan_started is None:
            self.scan_started,self.scan_last_yaw = monotonic,yaw
        else:
            delta = math.atan2(math.sin(yaw-self.scan_last_yaw),math.cos(yaw-self.scan_last_yaw))
            self.scan_progress += delta  # Signed: turning right cannot complete a left scan.
            self.scan_last_yaw = yaw
        if monotonic-self.scan_started > self.cfg.initial_scan_timeout:
            return self.halt('INITIAL_SCAN_TIMEOUT')
        if self.scan_progress >= self.cfg.initial_scan_angle:
            self.scan_complete = True
            self.goal_kind = 'random_frontier'
            return None
        base = self.grid.cell(base_matrix[:2,3])
        if not self.grid.reachable(base,allow_unknown=True)[base]:
            return self.halt('INITIAL_SCAN_BLOCKED')
        self.goal_kind = 'initial_scan'
        return Decision('INITIAL_SCAN','left',0.,self.cfg.turn_speed,
                        phase='INITIAL_SCAN',scan_progress_rad=self.scan_progress,
                        scan_target_rad=self.cfg.initial_scan_angle,goal_source='initial_scan')

    def state(self):
        return {'phase':'HALTED' if self.halted else 'EXPLORING' if self.scan_complete else 'INITIAL_SCAN',
                'scan_progress_rad':self.scan_progress,'scan_target_rad':self.cfg.initial_scan_angle,
                'exploration_policy':self.cfg.exploration_policy,'goal_kind':self.goal_kind,
                'goal_source':self.goal_kind,'planning_allows_unknown':self.cfg.exploration_policy=='paper_frontier'}

    def plan(self, base_matrix, monotonic):
        grid = self.grid
        base = grid.cell(base_matrix[:2,3])
        paper = self.cfg.exploration_policy == 'paper_frontier'
        reachable = grid.reachable(base,allow_unknown=paper)
        if not reachable[base]: return Decision('BASE_NOT_TRAVERSABLE')
        goals, edge = grid.paper_frontiers() if paper else grid.frontiers(reachable,base)
        self.frontier_goals,self.frontier_edge = goals,edge
        self.blacklist = {g:t for g,t in self.blacklist.items() if monotonic < t}
        semantic_goal = None
        if self.target_xy is not None:
            distance_to_target = np.linalg.norm(self.target_xy-base_matrix[:2,3])
            if distance_to_target <= self.cfg.target_approach_distance:
                return Decision('TARGET_REACHED',goal_xy=self.target_xy.tolist())
            # Optional detected-target approach retains measured free-space checks.
            cells = np.argwhere(grid.reachable(base))
            xy = grid.origin+(cells[:,::-1]+.5)*self.cfg.resolution
            distances = np.linalg.norm(xy-self.target_xy,axis=1)
            approach = ((distances >= self.cfg.target_approach_distance-.1)
                        & (distances <= self.cfg.target_approach_distance+.1))
            candidates_semantic = cells[approach]
            for blocked in self.blacklist:
                candidates_semantic = candidates_semantic[
                    np.linalg.norm((candidates_semantic-np.asarray(blocked))*self.cfg.resolution,
                                   axis=1) > self.cfg.goal_tolerance]
            if len(candidates_semantic):
                # Retain the chosen approach despite small target/mapping jitter.
                if self.goal_kind == 'semantic_target' and self.goal is not None and any(
                        np.array_equal(cell,self.goal) for cell in candidates_semantic):
                    semantic_goal = self.goal
                else:
                    index = np.argmin(np.linalg.norm(candidates_semantic-np.asarray(base),axis=1))
                    semantic_goal = tuple(int(v) for v in candidates_semantic[index])
        if semantic_goal is not None:
            if self.goal_kind != 'semantic_target' or self.goal != semantic_goal:
                self.goal = semantic_goal
                self.progress_at = monotonic
                self.best_distance = math.inf
            self.goal_kind = 'semantic_target'
        elif self.goal_kind == 'semantic_target':
            self.goal = None
            self.goal_kind = 'random_frontier'
        if self.goal is not None:
            distance = np.linalg.norm(grid.xy(self.goal)-base_matrix[:2,3])
            valid = reachable[self.goal] and (self.goal_kind in ('semantic_target','random_map_fallback')
                        or (self.goal in goals if paper else edge[self.goal]))
            if distance < self.best_distance-.03:
                self.best_distance, self.progress_at = distance, monotonic
            expired = monotonic-self.progress_at > self.cfg.goal_no_progress_sec
            if not valid or distance <= self.cfg.goal_tolerance or expired:
                if expired: self.blacklist[self.goal] = monotonic+self.cfg.goal_blacklist_sec
                semantic = self.goal_kind == 'semantic_target'
                arrival_xy = grid.xy(self.goal).tolist()
                self.goal = None
                if semantic and distance <= self.cfg.goal_tolerance:
                    return Decision('TARGET_APPROACH_REACHED',goal_xy=arrival_xy)
                if semantic and expired:
                    return Decision('TARGET_NO_PROGRESS',goal_xy=arrival_xy)
        candidates = [g for g in goals if g not in self.blacklist and reachable[g]
                      and np.linalg.norm(grid.xy(g)-base_matrix[:2,3]) > self.cfg.goal_tolerance]
        # Prefer a newly available frontier over continuing an unknown-map fallback.
        if candidates and self.goal_kind == 'random_map_fallback': self.goal = None
        if self.goal is None:
            if candidates:
                self.goal_kind = 'random_frontier'
                self.goal = candidates[int(self.rng.integers(len(candidates)))]
            elif paper:
                self.goal_kind = 'random_map_fallback'
                # Sample the whole map like upstream, with bounded rejection of
                # occupied/disconnected/arrived goals instead of forcing them free.
                for _ in range(self.cfg.fallback_sample_attempts):
                    g = tuple((self.rng.random(2)*(grid.n-1)).astype(int))
                    if (reachable[g] and g not in self.blacklist and
                            np.linalg.norm(grid.xy(g)-base_matrix[:2,3]) > self.cfg.goal_tolerance):
                        self.goal = g
                        break
                if self.goal is None:
                    return Decision('NO_VALID_RANDOM_GOAL',frontier_count=len(goals),
                                    goal_source=self.goal_kind,planning_allows_unknown=True)
            else:
                return Decision('NO_REACHABLE_FRONTIER', frontier_count=len(goals))
            self.best_distance = np.linalg.norm(grid.xy(self.goal)-base_matrix[:2,3])
            self.progress_at = monotonic
        goal_xy = grid.xy(self.goal).tolist()
        planner = self.fmm_class(reachable.astype(np.uint8), step_size=5)
        planner.set_goal(self.goal)
        row,col,replan,stop = planner.get_short_term_goal(base)
        stg = (int(row),int(col))
        if (replan or not (0<=stg[0]<grid.n and 0<=stg[1]<grid.n)
                or not all(reachable[r,c] for r,c in line(base,stg))):
            self.blacklist[self.goal] = monotonic+self.cfg.goal_blacklist_sec
            self.goal = None
            return Decision('FMM_NO_VALID_STEP', goal_xy=goal_xy, frontier_count=len(goals))
        if stop:
            if self.goal_kind != 'semantic_target':
                self.blacklist[self.goal] = monotonic+self.cfg.goal_blacklist_sec
            self.goal = None
            reason = ('TARGET_APPROACH_REACHED' if self.goal_kind == 'semantic_target' else
                      'RANDOM_GOAL_REACHED' if self.goal_kind == 'random_map_fallback' else 'FRONTIER_REACHED')
            return Decision(reason, goal_xy=goal_xy, frontier_count=len(goals))
        target = grid.xy(stg)
        delta = target-base_matrix[:2,3]
        yaw = math.atan2(base_matrix[1,0],base_matrix[0,0])
        error = math.atan2(math.sin(math.atan2(delta[1],delta[0])-yaw),
                           math.cos(math.atan2(delta[1],delta[0])-yaw))
        if error > self.cfg.turn_threshold: action,v,w = 'left',0.,self.cfg.turn_speed
        elif error < -self.cfg.turn_threshold: action,v,w = 'right',0.,-self.cfg.turn_speed
        else: action,v,w = 'forward',self.cfg.forward_speed,0.
        return Decision('PLANNED',action,v,w,goal_xy,target.tolist(),len(goals),
                        goal_source=self.goal_kind,planning_allows_unknown=paper,
                        scan_progress_rad=self.scan_progress,scan_target_rad=self.cfg.initial_scan_angle)

    def update(self, posed, monotonic, target_points=None):
        if self.halted: return Decision(self.halted,phase='HALTED')
        obs = posed.observation
        if obs.stamp_ns <= self.last_stamp: return Decision('NO_NEW_OBSERVATION')
        base, camera = rigid(posed.world_from_base), rigid(posed.world_from_camera)
        if target_points is not None:
            values = np.asarray(target_points,float)
            if values.ndim != 2 or values.shape[1] != 3 or not np.isfinite(values).all():
                raise ValueError('INVALID_TARGET_POINTS')
            if len(values)>=3: self.target_xy = np.median(values[:,:2],axis=0)

        if self.grid is None: self.grid = Grid(self.cfg,base[:2,3])
        points = unproject(obs.depth_m,np.asarray(obs.depth_info.k).reshape(3,3),self.cfg.pixel_stride)
        if not len(points): return Decision('NO_USABLE_DEPTH')
        world = points @ camera[:3,:3].T+camera[:3,3]
        try:
            ground = ground_point(base,self.cfg.base_height)[:3]
            self.grid.update(world,camera[:3,3],ground)
            result = self.initial_scan(base,monotonic)
            if result is None: result = self.plan(base,monotonic)
        except ValueError as exc:
            return self.halt(str(exc))
        self.last_stamp = obs.stamp_ns
        result.points = len(points)
        return result
