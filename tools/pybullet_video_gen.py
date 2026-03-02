import pybullet as p
import pybullet_data
import os
import numpy as np
from scipy.spatial.transform import Rotation as R
import json, csv
import cv2
import argparse
import yaml

# Camera parameters
fov = 60  # Field of view
aspect_ratio = 4.0/3.0
# Render distances
near_plane = 0.03
far_plane = 30

framerate = 60

def gen_positions_from_waypoints(lines):
    if (len(lines)) == 1:
        with open(lines[0], 'r') as file:
            print(file.read())
    #TODO generate video files from .json files of pathplanner

    p_x = np.array([])
    p_y = np.array([])
    p_z = np.array([])
    p_roll = np.array([])
    p_pitch = np.array([])
    p_yaw = np.array([])

    if float(lines[0][0]) != 0.0:
        print("error, first waypoint must start at 0.0 seconds")
        exit(-1)

    for i in range(len(lines)):
        if i == 0:
            continue
        n_frames = int((float(lines[i][0]) - float(lines[i-1][0])) * framerate)

        p_x = np.hstack((p_x, np.linspace(float(lines[i-1][1]), float(lines[i][1]), n_frames)))
        p_y = np.hstack((p_y, np.linspace(float(lines[i-1][2]), float(lines[i][2]), n_frames)))
        p_z = np.hstack((p_z, np.linspace(float(lines[i-1][3]), float(lines[i][3]), n_frames)))
        p_roll = np.hstack((p_roll, np.linspace(float(lines[i-1][4]), float(lines[i][4]), n_frames)))
        p_pitch = np.hstack((p_pitch, np.linspace(float(lines[i-1][5]), float(lines[i][5]), n_frames)))
        p_yaw = np.hstack((p_yaw, np.linspace(float(lines[i-1][6]), float(lines[i][6]), n_frames)))

    positions = np.stack([p_x, p_y, p_z, p_roll, p_pitch, p_yaw-90], axis=1)

    return positions

# Populate AprilTags from a .fmap file
def populate_apriltags(fmap_file):
    tags = []
    with open(fmap_file, "r") as f:
        fmap_data = json.load(f)

    offset_x = 0
    offset_y = 0
    if 'fieldlength' in fmap_data:
        offset_x = fmap_data['fieldlength'] / 2.0
        offset_y = fmap_data['fieldwidth'] / 2.0


    for tag in fmap_data["fiducials"]:
        id = tag["id"]
        transform = np.array(tag["transform"]).reshape((4,4))
        translation = transform[:3,-1].reshape(-1)
        rotation = R.from_matrix(transform[:3, :3]).as_matrix()

        translation[0] += offset_x
        translation[1] += offset_y

        b_tag = p.loadURDF("./at_objs/at.urdf", translation, R.from_matrix(rotation).as_quat())

        if (id < 10):
            id_str = "0000{}".format(id)
        else:
            id_str = "000{}".format(id)

        texture_id = p.loadTexture("./at_objs/apriltag-imgs/tag36h11/tag36_11_{}-big-cleaned.png".format(id_str))
        p.changeVisualShape(b_tag, -1, textureUniqueId=texture_id)


# Function to render the RGB image from the synthetic camera
def render_camera(position, cam_extrinsic):
    # Extract position and orientation components
    x, y, z, roll, pitch, yaw = position

    # Compute the rotation matrix and camera direction
    rotation = R.from_euler('yxz', [roll, pitch, yaw], degrees=True)
    rotation_matrix = rotation.as_matrix()

    e_roll, e_pitch, e_yaw = cam_extrinsic[3:]
    extrinsic_rotation = R.from_euler('yxz', [e_roll, e_pitch, e_yaw], degrees=True)
    rotation_matrix = rotation_matrix @ extrinsic_rotation.as_matrix()  #TODO these rotations are dependent, i.e. pitching and yawing induces a roll

    # Camera up vector and forward vector in local frame
    camera_forward_local = np.array([0, 1, 0])  # Y-axis is forward # TODO the x value that is detected is negative to what gets generated from the config yaml???
    camera_up_local = np.array([0, 0, 1])       # Z-axis is up

    # Transform to global frame
    camera_forward_global = rotation_matrix @ camera_forward_local
    camera_up_global = rotation_matrix @ camera_up_local

    camera_position = np.array([x, y, z])
    e_x, e_y, e_z = cam_extrinsic[:3]
    camera_position += [-e_x, -e_y, e_z]

    # Calculate camera target and up vector

    camera_target = camera_position + camera_forward_global
    camera_up = camera_up_global

    view_matrix = p.computeViewMatrix(camera_position, camera_target.tolist(), camera_up.tolist())  # Camera up vector
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect_ratio, near_plane, far_plane)

    width, height, rgb_img, _, _ = p.getCameraImage(
        width=640,
        height=480,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
    )
    return np.reshape(rgb_img, (height, width, 4))[:, :, :3]  # Return the RGB part of the image


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gui", help="display pybullet gui during render", action="store_true")
    # parser.add_argument("--user-control", help="user moves the camera around scene instead of following predetermined trajectory", action="store_true")
    # parser.add_argument("--waypoints", help="specify the .csv file with waypoints to generate the trajectory for the camera to follow", default="sim_waypoint_input.csv")
    # parser.add_argument("--fmap", help="specify the .fmap file with apriltag locations and orientations", default="../fmap/field.fmap")
    parser.add_argument("--config", help="specify the configuration file path", default="./sim-config.yml")
    parser.add_argument("--output", help="specify the output directory", default="./sim_output")

    args = parser.parse_args()

    # Connect to PyBullet physics server
    if (args.gui):
        p.connect(p.GUI)
    else:
        p.connect(p.DIRECT)

    # Parse the configuration file
    with open(args.config) as stream:
        try:
            config_yaml = yaml.safe_load(stream)
        except yaml.YAMLError as err:
            print(err)
            exit(-1)

    # Create the output directory
    if not os.path.exists(args.output):
        os.makedirs(args.output)

    for test in config_yaml["tests"]:
        test_name = test
        print("#################################################\nGenerating artifacts for {}\n#################################################".format(test_name))
        test = config_yaml["tests"][test]

        # Set up the physics simulation
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Path to PyBullet data files

        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        p.setGravity(0, 0, -9.8)

        # Load a flat plane and apriltags into the scene
        plane_id = p.loadURDF("plane.urdf")
        populate_apriltags(test["fmap"])

        # Setup recording
        camera_names = test["camera_names"]
        camera_extrinsics = test["camera_extrinsics"]
        video_filenames = [args.output + "/" + test_name[:-4] + "_" + f + ".mp4" for f in camera_names]

        video_writers = [cv2.VideoWriter(f, cv2.VideoWriter_fourcc(*'mp4v'), framerate, (640,480)) for f in video_filenames]

        with open(args.output + "/" + test_name[:-4] + "_gt.csv", "w") as f:

            progress_tracker=0
            time = 0
            positions = gen_positions_from_waypoints(test["waypoints"])
            for i, pos in enumerate(positions):

                for cam_index in range(len(video_filenames)):
                    cam_img = render_camera(pos, camera_extrinsics[cam_index])
                    video_writers[cam_index].write(cam_img)

                percent_complete = round(i/len(positions) * 100, 2)
                if (percent_complete >= progress_tracker):
                    progress_tracker += 10
                    print("{}: {}% complete".format(test_name, percent_complete))

                f.write(str(round(time, 9)*1.0e9) + "," + ",".join((str(x) for x in pos.tolist())) + "\n")

                time += 1.0/framerate

            for writer in video_writers:
                writer.release()

        print("{}: 100.0% complete".format(test_name))
    print("finished")

if __name__ == "__main__":
    main()