def __init__(self):
    super().__init__('zigzag_coverage_node')

    self.waypoint_pub = self.create_publisher(
        PointStamped,
        '/coverage/waypoint',
        10
    )

    qos = QoSProfile(depth=1)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

    self.path_pub = self.create_publisher(
        Path,
        '/coverage_path',
        qos
    )

    self.radius = 50.0
    self.center_x = 0.0
    self.center_y = 0.0
    self.spacing = 1.5   # try 1.5 first


    self.waypoints = self.generate_zigzag()
    self.get_logger().info(
        f"Generated {len(self.waypoints)} coverage waypoints"
    )
    self.index = 0

    self.publish_path()

    self.timer = self.create_timer(1.0, self.publish_waypoint)

    self.get_logger().info("Zig-Zag Coverage Planner started")
    