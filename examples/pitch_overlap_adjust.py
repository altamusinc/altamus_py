# run the following to make it work on wayland
# export XDG_SESSION_TYPE=x11

from pypcd4.pypcd4 import PointCloud
from altamus_py.scan import EOSV2Scan
import altamus_py.mavlink as mavlink
import numpy as np
import math
import open3d as o3d
import copy
import scipy
import json
import time
from pathlib import Path
from enum import Enum

fine_increment_amount = 0.1
coarse_increment_amount = 2

class IncrementDirection(Enum):
    POSITIVE = 0,
    NEGATIVE = 1,

class TargetParameter(Enum):
    ROLL = 0,
    PITCH = 1,

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4459,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=[1.6784, 2.0612, 1.4451],
                                      up=[-0.3402, -0.9189, -0.1996])


def output_hyperion_translations(input_transform: np.ndarray):
    translations = input_transform[:3, 3]
    rotation_matrix = input_transform[:3, :3]
    # weird memory bug, doing a copy seems to fix it
    rotation_matrix = rotation_matrix.copy()
    r = scipy.spatial.transform.Rotation.from_matrix(rotation_matrix)
    eulers = r.as_euler('xyz', degrees=True)
    x_rot, y_rot, z_rot = eulers[0], eulers[1], eulers[2]
    x_trans, y_trans, z_trans = translations[0], translations[1], translations[2]
    output_transform = {
        "rotate_x": x_rot.round(4),
        "rotate_y": y_rot.round(4),
        "rotate_z": z_rot.round(4),
        "translate_x": (x_trans * 12).round(4),
        "translate_y": (y_trans * 12).round(4),
        "translate_z": (z_trans * 12).round(4)
    }

    json_transform = json.dumps(output_transform)
    print(json_transform)

    return json_transform


def evaluate(scan: EOSV2Scan, visualize: bool = False) -> o3d.cpu.pybind.pipelines.registration.RegistrationResult:
    start_time = time.perf_counter()
    overlap, beginning, end = scan.yaw_overlap_points

    # find the matching yaw range in the "primary" portion and extract them into their own pcd
    # create a point cloud and assign the points from the np array
    primary_pcd = o3d.geometry.PointCloud()
    primary_pcd.points = o3d.utility.Vector3dVector(
        beginning[:, 0:3])  # slicing out just the first 3 (x,y,z)

    overlap_pcd = o3d.geometry.PointCloud()
    overlap_pcd.points = o3d.utility.Vector3dVector(end[:, 0:3])

    evaluation = o3d.pipelines.registration.evaluate_registration(
        primary_pcd, overlap_pcd, 5)
    print(f"{evaluation.inlier_rmse} \t transform: {scan.header.scan_transform}")
    if visualize:
        primary_pcd.paint_uniform_color([1, 0.706, 0])
        overlap_pcd.paint_uniform_color([0, 0.651, 0.929])
        o3d.visualization.draw_geometries([overlap_pcd, primary_pcd])
    return evaluation

def adjust_parameter(scan: EOSV2Scan, parameter: TargetParameter) -> float:
    scan = copy.deepcopy(scan) # copy the scan so we don't modify the header while we're working
    match parameter:
        case TargetParameter.PITCH:
            attr = "pitch_offset"
        case TargetParameter.ROLL:
            attr = "roll_offset"
    print(f"Adjusting {attr}")
    starting_transform = copy.deepcopy(scan.header.scan_transform)
    if starting_transform is None:
        print("no transform in the header, using a default value")
        starting_transform = mavlink.MAVLink_scan_transform_message(
            roll_offset=0, pitch_offset=0, pitch_scale=1.0, yaw_scale=1.0, range_scale=1.0, max_range=18000)

    initial_score = evaluate(scan)

    # try to pick a best direction
    positive_transform = copy.deepcopy(starting_transform)
    negative_transform = copy.deepcopy(starting_transform)
    setattr(positive_transform, attr, getattr(starting_transform, attr) + coarse_increment_amount)
    setattr(negative_transform, attr, getattr(starting_transform, attr) - coarse_increment_amount)

    scan.apply_new_transform_to_scan(positive_transform)
    positive_score = evaluate(scan)

    scan.apply_new_transform_to_scan(negative_transform)
    negative_score = evaluate(scan)

    print(f"initial_score: {initial_score.inlier_rmse}")
    print(f"positive_score: {positive_score.inlier_rmse}")
    print(f"negative_score: {negative_score.inlier_rmse}")

    working_transform = copy.deepcopy(starting_transform)
    if positive_score.inlier_rmse < negative_score.inlier_rmse:
        print("POSITIVE is best")
        direction = IncrementDirection.POSITIVE
        setattr(working_transform, attr, getattr(starting_transform, attr) - coarse_increment_amount)
    else:
        print("NEGATIVE is best")
        direction = IncrementDirection.NEGATIVE
        setattr(working_transform, attr, getattr(starting_transform, attr) + coarse_increment_amount)

    best_score = copy.deepcopy(initial_score)
    best_transform = copy.deepcopy(starting_transform)

    iterations = 100
    i = 0
    while i < iterations:
        if direction is IncrementDirection.POSITIVE:
            new_val = getattr(working_transform, attr) + fine_increment_amount
        else:
            new_val = getattr(working_transform, attr) - fine_increment_amount

        setattr(working_transform, attr, new_val)
        scan.apply_new_transform_to_scan(working_transform)
        score = evaluate(scan)
        if score.inlier_rmse < best_score.inlier_rmse:
            # print("new best!")
            best_score = copy.deepcopy(score)
            best_transform = copy.deepcopy(working_transform)
        i += 1

    print(f"Best Transform: {best_transform}")
    return getattr(best_transform, attr)

file = Path("./tests/sample_files/batch_plant.bin")
scan = EOSV2Scan.from_path(file)
print(f"Transform in binfile: {scan.header.scan_transform}")
transform = copy.deepcopy(scan.header.scan_transform)
# transform = mavlink.MAVLink_scan_transform_message(
#     roll_offset=0,
#     pitch_offset=0,
#     pitch_scale=1.0,
#     yaw_scale=1.0,
#     range_scale=1.0,
#     max_range=18000)
# print("Resetting to 0 for testing")
# scan.apply_new_transform_to_scan(transform=transform)

transform.pitch_offset = adjust_parameter(scan, TargetParameter.PITCH)
scan.apply_new_transform_to_scan(transform=transform)
# transform.roll_offset = adjust_parameter(scan, TargetParameter.ROLL)
# scan.apply_new_transform_to_scan(transform=transform)
# evaluate(scan, visualize=True)
print("end")
