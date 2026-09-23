#!/usr/bin/env python3
"""
DSG Mesh to 2D Map Node
========================
Extracts a 2D horizontal cross-section (slice) from a 3D Dynamic Scene Graph (DSG) Mesh
and publishes it as a ROS 2 nav_msgs/msg/OccupancyGrid map for AMCL localization and Nav2.

Features:
- Subscribes to DSG updates or loads local DSG JSON file.
- Fast vectorized 3D Mesh plane intersection at height Z = z_cut.
- Publishes OccupancyGrid on /map with TRANSIENT_LOCAL QoS (latched topic).
- Optional auto-export to map.yaml and map.pgm for ROS 2 map_server compatibility.
"""

import os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion

try:
    import spark_dsg
    SPARK_DSG_AVAILABLE = True
except ImportError:
    SPARK_DSG_AVAILABLE = False


def extract_2d_mesh_slice(vertices, faces, z_cut):
    """
    Extracts 2D line segments from 3D Mesh intersected by plane Z = z_cut.
    """
    if len(vertices) == 0 or len(faces) == 0:
        return np.empty((0, 2, 2))

    triangles = vertices[faces]  # (M, 3, 3)
    z_coords = triangles[:, :, 2]  # (M, 3)

    z_min = np.min(z_coords, axis=1)
    z_max = np.max(z_coords, axis=1)
    mask = (z_min <= z_cut) & (z_max >= z_cut) & (z_min < z_max)

    tris = triangles[mask]
    if len(tris) == 0:
        return np.empty((0, 2, 2))

    A, B, C = tris[:, 0, :], tris[:, 1, :], tris[:, 2, :]

    def intersect_edge(P1, P2):
        z1, z2 = P1[:, 2], P2[:, 2]
        denom = np.where((z2 - z1) == 0, 1e-8, z2 - z1)
        t = (z_cut - z1) / denom
        valid = (t >= 0.0) & (t <= 1.0) & (z1 != z2)
        P_int = P1[:, :2] + t[:, None] * (P2[:, :2] - P1[:, :2])
        return P_int, valid

    p_AB, v_AB = intersect_edge(A, B)
    p_BC, v_BC = intersect_edge(B, C)
    p_CA, v_CA = intersect_edge(C, A)

    segments = []
    for i in range(len(tris)):
        pts = []
        if v_AB[i]: pts.append(p_AB[i])
        if v_BC[i]: pts.append(p_BC[i])
        if v_CA[i]: pts.append(p_CA[i])
        if len(pts) >= 2:
            segments.append([pts[0], pts[1]])
            # # Lọc bỏ các đoạn thẳng nhiễu rác quá nhỏ (< 4cm)
            # if np.linalg.norm(pts[0] - pts[1]) >= 0.04:
            #     segments.append([pts[0], pts[1]])

    return np.array(segments)


def segments_to_occupancy_grid(segments_2d, resolution=0.05, padding=1.0):
    """
    Rasterizes 2D line segments into a 2D occupancy grid matrix.
    0 = free space, 100 = occupied (obstacle edge)
    """
    if len(segments_2d) == 0:
        return np.zeros((1, 1), dtype=np.int8), (0.0, 0.0)

    all_pts = segments_2d.reshape(-1, 2)
    x_min, y_min = np.min(all_pts, axis=0) - padding
    x_max, y_max = np.max(all_pts, axis=0) + padding

    width = int(np.ceil((x_max - x_min) / resolution))
    height = int(np.ceil((y_max - y_min) / resolution))

    width = max(10, min(4000, width))
    height = max(10, min(4000, height))

    grid = np.zeros((height, width), dtype=np.int8)

    for p1, p2 in segments_2d:
        c1 = int((p1[0] - x_min) / resolution)
        r1 = int((p1[1] - y_min) / resolution)
        c2 = int((p2[0] - x_min) / resolution)
        r2 = int((p2[1] - y_min) / resolution)

        n_pts = max(abs(c2 - c1), abs(r2 - r1)) + 1
        cols = np.linspace(c1, c2, n_pts).astype(int)
        rows = np.linspace(r1, r2, n_pts).astype(int)

        valid = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
        grid[rows[valid], cols[valid]] = 100

    return grid, (x_min, y_min)


class DsgMeshTo2DMapNode(Node):
    def __init__(self):
        super().__init__('dsg_mesh_to_2d_map_node')

        self.declare_parameter('dsg_filepath', '/home/tht/hydra_ws/src/planner/heracles/heracles/examples/scene_graphs/backend/dsg_with_mesh.json')
        self.declare_parameter('z_cut', 0.5)
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('padding', 1.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('save_yaml_path', '')

        self.dsg_filepath = self.get_parameter('dsg_filepath').get_parameter_value().string_value
        self.z_cut = self.get_parameter('z_cut').get_parameter_value().double_value
        self.resolution = self.get_parameter('resolution').get_parameter_value().double_value
        self.padding = self.get_parameter('padding').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.save_yaml_path = self.get_parameter('save_yaml_path').get_parameter_value().string_value

        # QoS for /map (TRANSIENT_LOCAL so new subscribers receive latest map)
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
        self.latest_grid_msg = None

        # Periodically publish / check for DSG updates
        timer_period = 1.0 / max(0.1, self.publish_rate)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(f"Initialized DSG Mesh to 2D Map Node (Z_cut={self.z_cut}m, resolution={self.resolution}m)")
        self.process_and_publish_map()

    def load_mesh_from_dsg(self):
        if not SPARK_DSG_AVAILABLE:
            self.get_logger().error("spark_dsg module is not available in Python environment!")
            return None, None

        target_file = self.dsg_filepath
        if not os.path.exists(target_file):
            candidates = [
                "/home/tht/hydra_ws/src/planner/heracles/heracles/examples/scene_graphs/backend/dsg_with_mesh.json",
                "/home/tht/hydra_ws/src/planner/heracles/heracles/examples/scene_graphs/frontend/dsg_with_mesh.json",
                "/home/tht/hydra_ws/src/planner/heracles/heracles/examples/scene_graphs/example_dsg.json"
            ]
            for c in candidates:
                if os.path.exists(c):
                    target_file = c
                    break

        if not os.path.exists(target_file):
            self.get_logger().error(f"DSG JSON file not found: {self.dsg_filepath}")
            return None, None

        try:
            dsg = spark_dsg.DynamicSceneGraph.load(target_file)
            if dsg.mesh is None:
                self.get_logger().warn(f"DSG loaded from {target_file} but mesh is None!")
                return None, None

            raw_v = dsg.mesh.get_vertices()
            raw_f = dsg.mesh.get_faces()

            if raw_v.shape[0] in [3, 6] and raw_v.shape[1] != 3:
                vertices = raw_v[:3, :].T
            else:
                vertices = raw_v

            if raw_f.shape[0] == 3 and raw_f.shape[1] != 3:
                faces = raw_f[:3, :].T
            else:
                faces = raw_f

            return vertices, faces
        except Exception as e:
            self.get_logger().error(f"Failed to load DSG file {target_file}: {e}")
            return None, None

    def process_and_publish_map(self):
        vertices, faces = self.load_mesh_from_dsg()
        if vertices is None or faces is None:
            return

        segments_2d = extract_2d_mesh_slice(vertices, faces, z_cut=self.z_cut)
        self.get_logger().info(f"Extracted {len(segments_2d)} 2D slice segments at Z={self.z_cut}m")

        grid_data, (x_min, y_min) = segments_to_occupancy_grid(
            segments_2d, resolution=self.resolution, padding=self.padding
        )

        height, width = grid_data.shape

        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.info.map_load_time = msg.header.stamp
        msg.info.resolution = float(self.resolution)
        msg.info.width = int(width)
        msg.info.height = int(height)

        msg.info.origin = Pose()
        msg.info.origin.position.x = float(x_min)
        msg.info.origin.position.y = float(y_min)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = grid_data.flatten().tolist()

        self.latest_grid_msg = msg
        self.map_pub.publish(msg)
        self.get_logger().info(f"Published 2D OccupancyGrid /map ({width}x{height} px, res={self.resolution}m) to frame '{self.frame_id}'")

        if self.save_yaml_path:
            self.save_map_to_disk(grid_data, (x_min, y_min), width, height)

    def save_map_to_disk(self, grid_data, origin, width, height):
        try:
            from PIL import Image
            pgm_path = self.save_yaml_path.replace('.yaml', '.pgm')
            # 255 = free (white), 0 = occupied (black)
            img_data = np.where(grid_data > 0, 0, 255).astype(np.uint8)
            img = Image.fromarray(img_data)
            img.save(pgm_path)

            yaml_content = f"""image: {os.path.basename(pgm_path)}
mode: trinary
resolution: {self.resolution}
origin: [{origin[0]}, {origin[1]}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
            with open(self.save_yaml_path, 'w') as f:
                f.write(yaml_content)
            self.get_logger().info(f"Saved map files to {self.save_yaml_path} and {pgm_path}")
        except Exception as e:
            self.get_logger().warn(f"Could not save map files: {e}")

    def timer_callback(self):
        if self.latest_grid_msg is not None:
            self.latest_grid_msg.header.stamp = self.get_clock().now().to_msg()
            self.map_pub.publish(self.latest_grid_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DsgMeshTo2DMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
