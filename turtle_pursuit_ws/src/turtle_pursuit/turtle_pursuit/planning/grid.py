import heapq, math

class GridPlanner:
    def __init__(self, width, height, blocked=()):
        self.width=width; self.height=height; self.blocked=set(blocked)
    def plan(self, start, goal):
        if start in self.blocked or goal in self.blocked: return []
        frontier=[(0.0,start)]; came={start:None}; cost={start:0.0}
        moves=((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1))
        while frontier:
            _, cur=heapq.heappop(frontier)
            if cur==goal: break
            for dx,dy in moves:
                nxt=(cur[0]+dx,cur[1]+dy)
                if not (0<=nxt[0]<self.width and 0<=nxt[1]<self.height) or nxt in self.blocked: continue
                nc=cost[cur]+math.hypot(dx,dy)
                if nc<cost.get(nxt,float('inf')):
                    cost[nxt]=nc; came[nxt]=cur
                    heapq.heappush(frontier,(nc+math.hypot(goal[0]-nxt[0],goal[1]-nxt[1]),nxt))
        if goal not in came: return []
        path=[]; cur=goal
        while cur is not None: path.append(cur); cur=came[cur]
        return path[::-1]

