"""Colour-matched ring pick-and-place task for a Franka Panda in CoppeliaSim."""

from dataclasses import dataclass
import math

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


@dataclass(frozen=True)
class TaskConfig:
    """Parameters for the scene-specific pick-and-place task."""

    move_duration: float = 5.0
    gripper_opening: float = 0.041
    border_clearance: float = 0.003
    pick_height_offset: float = 0.004
    safe_height_offset: float = 0.10
    transport_height_offset: float = 0.20
    place_height_offset: float = 0.09


@dataclass(frozen=True)
class SceneObjectPaths:
    """Object paths expected in a compatible CoppeliaSim scene."""

    target: str = ":/Target"
    gripper_tip: str = ":/Tip"
    left_gripper_joint: str = ":/panda_finger_joint1"
    right_gripper_joint: str = ":/panda_finger_joint2"

    def ring(self, color):
        """Return the scene path for a colour-coded ring."""
        return f":/{color.title()}_ring"

    def peg(self, color):
        """Return the scene path for a colour-coded peg."""
        return f":/{color.title()}_peg"


COLORS = ("red", "blue", "yellow")
CONFIG = TaskConfig()
SCENE_OBJECTS = SceneObjectPaths()


def lerp_3d(start, end, alpha):
    """Linearly interpolate between two Cartesian positions."""
    alpha = max(0.0, min(1.0, alpha))
    return [start[index] + (end[index] - start[index]) * alpha for index in range(3)]


def distance_3d(first, second):
    """Return Euclidean distance between two Cartesian positions."""
    return math.sqrt(sum((first[index] - second[index]) ** 2 for index in range(3)))


class PandaPickAndPlace:
    """Controls the task while keeping CoppeliaSim scene access in one place."""

    def __init__(self, sim, client, config=CONFIG, scene_objects=SCENE_OBJECTS):
        self.sim = sim
        self.client = client
        self.config = config
        self.scene_objects = scene_objects

        self.target = sim.getObject(scene_objects.target)
        self.gripper_tip = sim.getObject(scene_objects.gripper_tip)
        self.gripper_joints = [
            sim.getObject(scene_objects.left_gripper_joint),
            sim.getObject(scene_objects.right_gripper_joint),
        ]
        self.rings = {color: sim.getObject(scene_objects.ring(color)) for color in COLORS}
        self.pegs = {color: sim.getObject(scene_objects.peg(color)) for color in COLORS}

    def set_gripper_opening(self, opening):
        """Command the two gripper fingers, constrained to the scene's range."""
        opening = max(0.0, min(self.config.gripper_opening, opening))
        for joint in self.gripper_joints:
            self.sim.setJointTargetPosition(joint, opening)

    def open_gripper(self):
        self.set_gripper_opening(self.config.gripper_opening)

    def close_gripper(self):
        self.set_gripper_opening(0.0)

    def move_target(self, start, end):
        """Move the IK target along a linear Cartesian trajectory."""
        start_time = self.sim.getSimulationTime()
        end_time = start_time + self.config.move_duration
        while self.sim.getSimulationTime() < end_time:
            elapsed = self.sim.getSimulationTime() - start_time
            position = lerp_3d(start, end, elapsed / self.config.move_duration)
            self.sim.setObjectPosition(self.target, position, -1)
            self.client.step()
        self.sim.setObjectPosition(self.target, end, -1)
        self.client.step()

    def wait(self, seconds):
        """Advance the stepped simulation for a fixed duration."""
        end_time = self.sim.getSimulationTime() + seconds
        while self.sim.getSimulationTime() < end_time:
            self.client.step()

    def ring_radius(self, ring):
        """Estimate the outer radius from the ring's axis-aligned bounding box."""
        size_x = self.sim.getObjectFloatParam(ring, self.sim.objfloatparam_objbbox_max_x)
        size_x -= self.sim.getObjectFloatParam(ring, self.sim.objfloatparam_objbbox_min_x)
        size_y = self.sim.getObjectFloatParam(ring, self.sim.objfloatparam_objbbox_max_y)
        size_y -= self.sim.getObjectFloatParam(ring, self.sim.objfloatparam_objbbox_min_y)
        return max(size_x, size_y) / 2.0

    def task_poses(self, color):
        """Compute safe pick and placement poses for a colour-matched ring and peg."""
        ring_position = self.sim.getObjectPosition(self.rings[color], self.sim.handle_world)
        peg_position = self.sim.getObjectPosition(self.pegs[color], self.sim.handle_world)
        radius = self.ring_radius(self.rings[color])

        # Pick at the -Y outer edge. Retaining this offset aligns the ring at release.
        pick_position = [
            ring_position[0],
            ring_position[1] - radius - self.config.border_clearance,
            ring_position[2] + self.config.pick_height_offset,
        ]
        grasp_offset = [
            pick_position[0] - ring_position[0],
            pick_position[1] - ring_position[1],
        ]
        above_ring = [
            pick_position[0],
            pick_position[1],
            ring_position[2] + self.config.safe_height_offset,
        ]
        above_peg = [
            peg_position[0] + grasp_offset[0],
            peg_position[1] + grasp_offset[1],
            peg_position[2] + self.config.transport_height_offset,
        ]
        place_position = [
            peg_position[0] + grasp_offset[0],
            peg_position[1] + grasp_offset[1],
            peg_position[2] + self.config.place_height_offset,
        ]
        return above_ring, pick_position, above_peg, place_position

    def attach_ring(self, ring):
        """Use CoppeliaSim parenting as a simplified simulated grasp."""
        self.sim.setObjectParent(ring, self.gripper_tip, True)
        self.sim.setObjectInt32Param(ring, self.sim.shapeintparam_static, 1)
        self.sim.resetDynamicObject(ring)

    def release_ring(self, ring):
        """Release a ring from the simulated grasp."""
        self.sim.setObjectParent(ring, -1, True)
        self.sim.setObjectInt32Param(ring, self.sim.shapeintparam_static, 0)
        self.sim.resetDynamicObject(ring)

    def execute_ring_task(self, color, current_position):
        """Execute the complete pick-and-place sequence for one ring."""
        above_ring, pick_position, above_peg, place_position = self.task_poses(color)
        ring = self.rings[color]
        self.move_target(current_position, above_ring)
        self.move_target(above_ring, pick_position)
        self.close_gripper()
        self.wait(1.0)
        self.attach_ring(ring)
        self.wait(1.0)
        self.move_target(pick_position, above_peg)
        self.move_target(above_peg, place_position)
        self.release_ring(ring)
        self.open_gripper()
        self.wait(1.0)
        return place_position

    def choose_nearest_ring(self, current_position, remaining_colors):
        """Select the remaining ring closest to its pick pose."""
        distances = {
            color: distance_3d(current_position, self.task_poses(color)[1])
            for color in remaining_colors
        }
        return min(distances, key=distances.get)

    def run(self, first_color):
        """Place the first selected ring, then apply greedy nearest-neighbour ordering."""
        remaining_colors = list(COLORS)
        current_position = self.sim.getObjectPosition(self.target, self.sim.handle_world)
        selected_color = first_color
        while remaining_colors:
            current_position = self.execute_ring_task(selected_color, current_position)
            remaining_colors.remove(selected_color)
            if remaining_colors:
                selected_color = self.choose_nearest_ring(current_position, remaining_colors)


def choose_first_color():
    """Read and validate the first ring selected by the user."""
    color = input("Choose first ring color (red, blue, yellow): ").strip().lower()
    while color not in COLORS:
        color = input("Invalid color. Choose red, blue, or yellow: ").strip().lower()
    return color


def main():
    client = RemoteAPIClient()
    sim = client.require("sim")
    client.setStepping(True)
    controller = PandaPickAndPlace(sim, client)
    try:
        sim.startSimulation()
        controller.open_gripper()
        controller.run(choose_first_color())
        print("All rings positioned.")
    finally:
        sim.stopSimulation()


if __name__ == "__main__":
    main()
