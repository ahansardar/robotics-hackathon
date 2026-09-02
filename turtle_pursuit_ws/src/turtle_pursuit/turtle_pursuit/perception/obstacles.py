from collections import deque
import math


class ObstacleMapper:
    """Build a ray-cleared 2-D occupancy map from world-projected lidar scans."""

    def __init__(self, arena_half=5.0, resolution=.15, max_range=4.5,
                 wall_margin=.35, min_cluster_cells=4, min_range=.12,
                 cell_ttl=15.0, opponent_exclusion=.48,
                 max_component_span=2.2, bridge_gap_cells=2):
        self.arena_half=arena_half
        self.resolution=resolution
        self.max_range=max_range
        self.wall_margin=wall_margin
        self.min_cluster_cells=min_cluster_cells
        self.min_range=min_range
        self.cell_ttl=cell_ttl
        self.opponent_exclusion=opponent_exclusion
        self.max_component_span=max_component_span
        self.bridge_gap_cells=bridge_gap_cells
        self.cells={}; self.update_count=0

    @staticmethod
    def _ray_cells(start, end):
        """Return grid cells crossed by a beam, including both endpoints."""
        x0,y0=start; x1,y1=end
        dx=abs(x1-x0); dy=abs(y1-y0)
        sx=1 if x0<x1 else -1; sy=1 if y0<y1 else -1
        error=dx-dy; cells=[]
        while True:
            cells.append((x0,y0))
            if x0==x1 and y0==y1:
                return cells
            twice=2*error
            if twice>-dy:
                error-=dy; x0+=sx
            if twice<dx:
                error+=dx; y0+=sy

    def update(self, pose, ranges, angle_min, angle_increment, exclude=(), stamp=None):
        self.update_count+=1
        now=float(stamp if stamp is not None else self.update_count)
        self.cells={cell:last_seen for cell,last_seen in self.cells.items()
                    if now-last_seen<=self.cell_ttl}
        origin=(round(pose.x/self.resolution),round(pose.y/self.resolution))
        free_cells=set(); occupied_cells=set(); previous=None
        for index,value in enumerate(ranges):
            if math.isnan(value) or value<=self.min_range:
                previous=None
                continue
            angle=pose.yaw+angle_min+index*angle_increment
            has_hit=math.isfinite(value) and value<self.max_range
            beam_length=value if has_hit else self.max_range
            x=pose.x+beam_length*math.cos(angle)
            y=pose.y+beam_length*math.sin(angle)
            cell=(round(x/self.resolution),round(y/self.resolution))

            # Every beam is evidence that the cells before its return are empty.
            # No-return beams therefore erase a removed or relocated obstacle
            # immediately instead of waiting for its occupancy TTL to expire.
            ray=self._ray_cells(origin,cell)
            free_cells.update(ray[:-1] if has_hit else ray)

            inside=abs(x)<=self.arena_half-self.wall_margin and abs(y)<=self.arena_half-self.wall_margin
            is_opponent=any(math.hypot(x-p.x,y-p.y)<self.opponent_exclusion
                            for p in exclude if p is not None)
            if not has_hit or not inside or is_opponent:
                previous=None
                continue
            occupied_cells.add(cell)
            # Adjacent rays on one physical face can quantize with a one-cell gap.
            if previous is not None and max(abs(cell[0]-previous[0]),abs(cell[1]-previous[1]))<=self.bridge_gap_cells:
                occupied_cells.add(((cell[0]+previous[0])//2,(cell[1]+previous[1])//2))
            previous=cell

        # Protect all returns in this scan from free rays in adjacent beams.
        for cell in free_cells-occupied_cells:
            self.cells.pop(cell,None)
        for cell in occupied_cells:
            self.cells[cell]=now
        return self.centers()

    def centers(self):
        remaining=set(self.cells); components=[]
        while remaining:
            start=remaining.pop(); component={start}; queue=deque([start])
            while queue:
                x,y=queue.popleft()
                for dx in (-1,0,1):
                    for dy in (-1,0,1):
                        neighbor=(x+dx,y+dy)
                        if neighbor in remaining:
                            remaining.remove(neighbor); component.add(neighbor); queue.append(neighbor)
            if len(component)>=self.min_cluster_cells:
                xs=[c[0]*self.resolution for c in component]
                ys=[c[1]*self.resolution for c in component]
                width=max(xs)-min(xs); height=max(ys)-min(ys)
                # Arena obstacles are compact; reject accumulated streaks and artifacts.
                if width<=self.max_component_span and height<=self.max_component_span:
                    components.append(((min(xs)+max(xs))/2,(min(ys)+max(ys))/2))
        return sorted(components)
