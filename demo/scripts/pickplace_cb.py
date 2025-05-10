#! /usr/bin/env python3

from py_binding_tools import roscpp_init
from moveit.task_constructor import core, stages
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist, Vector3Stamped, Vector3
from std_msgs.msg import Header
from math import pi
import time


def init_robot():
    roscpp_init("mtc_tutorial")
    return "panda_arm", "hand"


def setup_planning_scene():
    object_name = "grasp_object"
    psi = PlanningSceneInterface(synchronous=True)
    psi.remove_world_object()

    objectPose = PoseStamped()
    objectPose.header.frame_id = "world"
    objectPose.pose.orientation.x = 1.0
    objectPose.pose.position.x = 0.30702
    objectPose.pose.position.y = 0.0
    objectPose.pose.position.z = 0.285

    psi.add_box(object_name, objectPose, size=[0.1, 0.05, 0.03])
    return object_name, objectPose


def create_multi_planner():
    cartesian_planner = core.CartesianPath()
    joint_planner = core.JointInterpolationPlanner()
    ompl_planner = core.PipelinePlanner("ompl")
    ompl_planner.planner = "RRTConnect"
    multi_planner = core.MultiPlanner()
    multi_planner.add(ompl_planner, cartesian_planner, joint_planner)
    multi_planner.max_velocity_scaling_factor = 0.1
    multi_planner.max_acceleration_scaling_factor = 0.1
    return multi_planner


def create_cartesian_motions(task, group, planner):
    header = Header(frame_id="world")

    # Move along x
    move = stages.MoveRelative("x +0.2", planner)
    move.group = group
    move.setDirection(Vector3Stamped(header=header, vector=Vector3(0.2, 0, 0)))
    task.add(move)

    # Move along y
    move = stages.MoveRelative("y -0.3", planner)
    move.group = group
    move.setDirection(Vector3Stamped(header=header, vector=Vector3(0, -0.3, 0)))
    task.add(move)

    # Rotate about z
    move = stages.MoveRelative("rz +45°", planner)
    move.group = group
    move.setDirection(TwistStamped(header=header, twist=Twist(angular=Vector3(0, 0, pi / 4.0))))
    task.add(move)

    # Joint-space offset
    move = stages.MoveRelative("joint offset", planner)
    move.group = group
    move.setDirection(dict(panda_joint1=pi / 6, panda_joint3=-pi / 6))
    task.add(move)


def create_pick_stage(task, arm, eef, object_name, planners, manual_pick_planner=False):
    planners_by_group = [(arm, planners)]
    task.add(stages.CurrentState("current"))
    task.add(stages.Connect("connect1", planners_by_group))

    grasp_generator = stages.GenerateGraspPose("Generate Grasp Pose")
    grasp_generator.angle_delta = 0.2
    grasp_generator.pregrasp = "open"
    grasp_generator.grasp = "close"
    grasp_generator.setMonitoredStage(task["current"])

    simpleGrasp = stages.SimpleGrasp(grasp_generator, "Grasp")
    ik_frame = PoseStamped()
    ik_frame.header.frame_id = "panda_hand"
    ik_frame.pose.position.z = 0.1034
    simpleGrasp.setIKFrame(ik_frame)

    if manual_pick_planner:
        cartesian_planner = core.CartesianPath()
        cartesian_planner.max_velocity_scaling_factor = 0.01
        cartesian_planner.max_acceleration_scaling_factor = 0.01
        cartesian_planner.step_size = 0.01
        cartesian_planner.jump_threshold = 0.0
        print(f"Python - Created cartesian_planner with max_velocity_scaling_factor: {cartesian_planner.max_velocity_scaling_factor}")
    else:
        cartesian_planner = None
        print("Python - Not using a manual cartesian planner (passing None)")
    pick = stages.Pick(simpleGrasp, "Pick", cartesian_planner)
    pick.eef = eef
    pick.object = object_name

    print(f"pick using manual_pick_planner: {manual_pick_planner}")

    # Access cartesianSolver() directly and check/modify its properties
    solver = pick.cartesianSolver()
    print(f"Original cartesianSolver max_velocity_scaling_factor: {solver.max_velocity_scaling_factor}")
    print(f"Original cartesianSolver max_acceleration_scaling_factor: {solver.max_acceleration_scaling_factor}")

    # Modify the solver properties directly
    solver.max_velocity_scaling_factor = 0.05
    solver.max_acceleration_scaling_factor = 0.05

    # Check that the changes were applied
    print(f"Updated cartesianSolver max_velocity_scaling_factor: {solver.max_velocity_scaling_factor}")
    print(f"Updated cartesianSolver max_acceleration_scaling_factor: {solver.max_acceleration_scaling_factor}")

    approach = TwistStamped()
    approach.header.frame_id = "world"
    approach.twist.linear.z = -1.0
    pick.setApproachMotion(approach, 0.03, 0.1)

    lift = TwistStamped()
    lift.header.frame_id = "panda_hand"
    lift.twist.linear.z = -1.0
    pick.setLiftMotion(lift, 0.03, 0.1)

    task.add(pick)
    task.add(stages.Connect("connect2", planners_by_group))


def create_place_stage(task, eef, object_name, objectPose):
    placePose = objectPose
    placePose.pose.position.y += 0.2

    place_generator = stages.GeneratePlacePose("Generate Place Pose")
    place_generator.setMonitoredStage(task["Pick"])
    place_generator.object = object_name
    place_generator.pose = placePose

    simpleUnGrasp = stages.SimpleUnGrasp(place_generator, "UnGrasp")

    place = stages.Place(simpleUnGrasp, "Place")
    place.eef = eef
    place.object = object_name
    place.eef_frame = "panda_link8"

    print("Place stage created without manual cartesian planner")

    retract = TwistStamped()
    retract.header.frame_id = "world"
    retract.twist.linear.z = 1.0
    place.setRetractMotion(retract, 0.03, 0.1)

    placeMotion = TwistStamped()
    placeMotion.header.frame_id = "panda_hand"
    placeMotion.twist.linear.z = 1.0
    place.setPlaceMotion(placeMotion, 0.03, 0.1)

    task.add(place)


def main():
    arm, eef = init_robot()
    object_name, objectPose = setup_planning_scene()

    task = core.Task()
    task.name = "cartesian + pick + place"

    multi_planner = create_multi_planner()

    # Add Cartesian motions
    # create_cartesian_motions(task, arm, multi_planner)

    # Add pick and place stages
    create_pick_stage(task, arm, eef, object_name, multi_planner, manual_pick_planner=True)
    create_place_stage(task, eef, object_name, objectPose)

    input("Press Enter to continue...")
    if task.plan():
        task.publish(task.solutions[0])

    time.sleep(3600)


if __name__ == "__main__":
    main()
