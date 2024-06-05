import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import heapq

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/occupancy_map',
            self.map_callback,
            10)
        self.publisher = self.create_publisher(
            Path,
            '/path',
            10)
        self.grid_map = None
        self.resolution = None
        self.map_origin = None
        print("tahap 1")

    def map_callback(self, msg):
        print("tahap2")
        self.grid_map = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        start =(14,7) 
        goal = (13,2)
        path = self.astar(start, goal)
        self.publish_path(path)

    def astar(self, start, goal):
        print("tahap 3")
        neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        close_set = set()
        came_from = {}
        gscore = {start: 0}
        fscore = {start: self.heuristic(start, goal)}
        oheap = []
        heapq.heappush(oheap, (fscore[start], start))
        counter=0
        
        while oheap:
            counter=counter+1
            print(counter)
            current = heapq.heappop(oheap)[1]
            print(current)
            if current == goal:
                data = []
                while current in came_from:
                    data.append(current)
                    current = came_from[current]
                return data
            close_set.add(current)
            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j
                tentative_g_score = gscore[current] + self.heuristic(current, neighbor)
                if 0 <= neighbor[0] < self.grid_map.shape[0]:
                    if 0 <= neighbor[1] < self.grid_map.shape[1]:
                        if self.grid_map[neighbor[0], neighbor[1]] == 100:
                            continue
                    else:
                        continue
                else:
                    continue
                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                    continue
                if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))
        return False

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def publish_path(self, path):
        print("tahap 4")
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        print(path)
        for p in path:
            pose = PoseStamped()
            pose.pose.position.x = p[0] * self.resolution 
            pose.pose.position.y = p[1] * self.resolution 
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.publisher.publish(path_msg)
        print('Path published')

def main(args=None):
    rclpy.init(args=args)
    astar_planner = AStarPlanner()
    rclpy.spin(astar_planner)
    astar_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
