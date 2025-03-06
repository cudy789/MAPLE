from direct.showbase.ShowBase import ShowBase
from panda3d.core import WindowProperties, NodePath, Camera, PerspectiveLens, Vec3
from direct.actor.Actor import Actor
from direct.task import Task

class RobotSimulation(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)

        # Load environment
        self.environ = self.loader.loadModel("models/environment")
        self.environ.reparentTo(self.render)
        self.environ.setScale(0.25, 0.25, 0.25)
        self.environ.setPos(-8, 42, 0)

        # Load robot model
        self.robot = Actor("models/panda")  # Replace with an actual robot model
        self.robot.reparentTo(self.render)
        self.robot.setScale(0.5)
        self.robot.setPos(0, 0, 0)

        # Movement variables
        self.robot_speed = 5.0
        self.turn_speed = 50.0
        self.movement = {"up": False, "down": False, "left": False, "right": False}

        # Set up cameras
        self.setup_cameras()

        # Accept key inputs
        self.accept("arrow_up", self.set_movement, ["up", True])
        self.accept("arrow_up-up", self.set_movement, ["up", False])
        self.accept("arrow_down", self.set_movement, ["down", True])
        self.accept("arrow_down-up", self.set_movement, ["down", False])
        self.accept("arrow_left", self.set_movement, ["left", True])
        self.accept("arrow_left-up", self.set_movement, ["left", False])
        self.accept("arrow_right", self.set_movement, ["right", True])
        self.accept("arrow_right-up", self.set_movement, ["right", False])

        # Add update loop
        self.taskMgr.add(self.update_robot, "update_robot_task")

    def set_movement(self, direction, state):
        """Updates movement dictionary when keys are pressed or released."""
        self.movement[direction] = state

    def setup_cameras(self):
        """Sets up third-person and first-person cameras with viewports."""
        # Third-person main camera
        self.main_camera = self.cam
        self.main_camera.setPos(0, -10, 5)
        self.main_camera.lookAt(self.robot)

        # Camera positions relative to the robot
        cam_positions = [
            Vec3(0, 1, 1),   # Front camera
            Vec3(-1, -1, 1),  # Left camera
            Vec3(1, -1, 1)    # Right camera
        ]

        # Create new display regions for FPV cameras
        display_regions = [(0.7, 1, 0.7, 1),  # Top-right
                           (0.7, 1, 0.4, 0.7),  # Middle-right
                           (0.7, 1, 0.1, 0.4)]  # Bottom-right

        self.robot_cameras = []

        for i, pos in enumerate(cam_positions):
            # Create a camera node
            cam_node = NodePath(Camera(f"robot_cam_{i}"))
            cam_node.reparentTo(self.robot)
            cam_node.setPos(pos)
            cam_node.lookAt(pos + Vec3(0, 2, 0))  # Look forward

            # Set perspective lens
            lens = PerspectiveLens()
            lens.setFov(60)
            cam_node.node().setLens(lens)

            # Attach camera to a new display region
            dr = self.win.makeDisplayRegion(*display_regions[i])
            dr.setSort(20 + i)  # Ensure it's above the main camera
            dr.setCamera(cam_node)

            self.robot_cameras.append(cam_node)

    def update_robot(self, task):
        """Applies movement and rotation based on user input each frame."""
        dt = globalClock.getDt()

        if self.movement["up"]:
            self.robot.setPos(self.robot, Vec3(0, self.robot_speed * dt, 0))
        if self.movement["down"]:
            self.robot.setPos(self.robot, Vec3(0, -self.robot_speed * dt, 0))
        if self.movement["left"]:
            self.robot.setH(self.robot.getH() + self.turn_speed * dt)
        if self.movement["right"]:
            self.robot.setH(self.robot.getH() - self.turn_speed * dt)

        return Task.cont

# Run the simulation
app = RobotSimulation()
app.run()
