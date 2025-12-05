import os
import tempfile
from queue import Empty, Queue
import time
from collections import defaultdict
from copy import deepcopy
import numpy as np
import cv2
from PIL import Image
import h5py
import imageio

from dex_control.data.image_transform import ImageTransformer
from dex_control.data.timeste_processer import TimestepProcessor
from dex_control.camera.multi_camera_wrapper import RecordedMultiCameraWrapper
from dex_control.utils.misc.parameters import camera_type_to_string_dict
from dex_control.utils.misc.misc_utils import time_ms, run_threaded_command

##############################################################

def write_dict_to_hdf5(hdf5_file, data_dict, keys_to_ignore=["image", "depth", "pointcloud"]):
    for key in data_dict.keys():
        # Pass Over Specified Keys #
        if key in keys_to_ignore:
            continue

        # Examine Data #
        curr_data = data_dict[key]
        if type(curr_data) == list:
            curr_data = np.array(curr_data)
        dtype = type(curr_data)

        # Unwrap If Dictionary #
        if dtype == dict:
            if key not in hdf5_file:
                hdf5_file.create_group(key)
            write_dict_to_hdf5(hdf5_file[key], curr_data)
            continue

        # Make Room For Data #
        if key not in hdf5_file:
            if dtype != np.ndarray:
                dshape = ()
            else:
                dtype, dshape = curr_data.dtype, curr_data.shape
            hdf5_file.create_dataset(key, (1, *dshape), maxshape=(None, *dshape), dtype=dtype)
        else:
            hdf5_file[key].resize(hdf5_file[key].shape[0] + 1, axis=0)

        # Save Data #
        hdf5_file[key][-1] = curr_data


class TrajectoryWriter:
    def __init__(self, filepath, metadata=None, exists_ok=False, post_process=False):
        assert (not os.path.isfile(filepath)) or exists_ok
        self._filepath = filepath
        self._dirpath = os.path.dirname(filepath)
        self._hdf5_file = h5py.File(filepath, "w")
        self._queue_dict = defaultdict(Queue)
        self._open = True

        self.post_process = post_process
        if self.post_process:
            image_transform_kwargs = {"remove_alpha": True, "bgr_to_rgb": True, "augment": False}
            self._timestep_processor = TimestepProcessor(
                camera_extrinsics=["fixed_camera", "hand_camera", "varied_camera"],
                image_transform_kwargs=image_transform_kwargs,
            )

            os.makedirs(os.path.join(self._dirpath, "recordings"), exist_ok=True)
            self._video_writers = {}
            self._npz_data = {"states": [], "actions_pos": [], "actions_vel": []}
            self.t = {}

        # Add Metadata #
        if metadata is not None:
            self._update_metadata(metadata)

        # Start HDF5 Writer Thread #
        def hdf5_writer(data):
            return write_dict_to_hdf5(self._hdf5_file, data)
        run_threaded_command(self._write_from_queue, args=(hdf5_writer, self._queue_dict["hdf5"]))

    def write_timestep(self, timestep):
        self._queue_dict["hdf5"].put(timestep)
        if self.post_process:
            if not timestep["observation"]["timestamp"]["skip_action"]:
                timestep = self._timestep_processor.forward(timestep)
                self._update_npz_data(timestep)
                self._update_video_files(timestep)

    def _update_metadata(self, metadata):
        for key in metadata:
            self._hdf5_file.attrs[key] = deepcopy(metadata[key])

    def _write_from_queue(self, writer, queue):
        while self._open:
            try:
                data = queue.get(timeout=1)
            except Empty:
                continue
            writer(data)
            queue.task_done()
    
    def _update_video_files(self, timestep):
        image_dict = self._timestep_processor.get_image_dict(timestep)

        for video_id, (img, _) in image_dict.items():
            if video_id not in self._video_writers:
                filename = os.path.join(self._dirpath, "recordings", f"{video_id}.mp4")
                self._video_writers[video_id] = imageio.get_writer(filename, fps=15, macro_block_size=1)
                run_threaded_command(
                    self._write_from_queue, args=(self._video_writers[video_id].append_data, self._queue_dict[video_id])
                )
            if video_id not in self.t:
                self.t[video_id] = 0
                os.makedirs(os.path.join(self._dirpath, "recordings", "frames", video_id), exist_ok=True)

            self._queue_dict[video_id].put(img)
            Image.fromarray(img[:, :, :3]).save(os.path.join(self._dirpath, "recordings", "frames", video_id, f"{self.t[video_id]:05d}.jpg"))
            self.t[video_id] += 1

    
    def _update_npz_data(self, timestep):
        self._npz_data["states"].append(timestep["observation"]["state"])
        self._npz_data["actions_pos"].append(timestep["action"]["cartesian_position"])
        self._npz_data["actions_vel"].append(timestep["action"]["cartesian_velocity"])

    def close(self, metadata=None):
        if metadata is not None:
            self._update_metadata(metadata)

        # Finish Remaining Jobs #
        [queue.join() for queue in self._queue_dict.values()]

        if self.post_process:
            for video_id in self._video_writers:
                self._video_writers[video_id].close()
            self._video_writers.clear()
            self._npz_data = {k: np.array(v) for k, v in self._npz_data.items()}
            np.savez(os.path.join(self._dirpath, "trajectory.npz"), **self._npz_data)

        # Close File #
        self._hdf5_file.close()
        self._open = False


##############################################################

def create_video_file(suffix=".mp4", byte_contents=None):
    # Create Temporary File #
    temp_file = tempfile.NamedTemporaryFile(suffix=suffix)
    filename = temp_file.name

    # If Byte Contents Provided, Write To File #
    if byte_contents is not None:
        with open(filename, "wb") as binary_file:
            binary_file.write(byte_contents)

    return filename


def get_hdf5_length(hdf5_file, keys_to_ignore=[]):
    length = None

    for key in hdf5_file.keys():
        if key in keys_to_ignore:
            continue

        curr_data = hdf5_file[key]
        if isinstance(curr_data, h5py.Group):
            curr_length = get_hdf5_length(curr_data, keys_to_ignore=keys_to_ignore)
        elif isinstance(curr_data, h5py.Dataset):
            curr_length = len(curr_data)
        else:
            raise ValueError

        if length is None:
            length = curr_length
        assert curr_length == length

    return length


def load_hdf5_to_dict(hdf5_file, index, keys_to_ignore=[]):
    data_dict = {}

    for key in hdf5_file.keys():
        if key in keys_to_ignore:
            continue

        curr_data = hdf5_file[key]
        if isinstance(curr_data, h5py.Group):
            data_dict[key] = load_hdf5_to_dict(curr_data, index, keys_to_ignore=keys_to_ignore)
        elif isinstance(curr_data, h5py.Dataset):
            data_dict[key] = curr_data[index]
        else:
            raise ValueError

    return data_dict


class TrajectoryReader:
    def __init__(self, filepath, read_images=True):
        self._hdf5_file = h5py.File(filepath, "r")
        is_video_folder = "observations/videos" in self._hdf5_file
        self._read_images = read_images and is_video_folder
        self._length = get_hdf5_length(self._hdf5_file, keys_to_ignore=["videos"])
        self._video_readers = {}
        self._index = 0

    def length(self):
        return self._length

    def read_timestep(self, index=None, keys_to_ignore=[]):
        # Make Sure We Read Within Range #
        if index is None:
            index = self._index
        else:
            assert not self._read_images
            self._index = index
        assert index < self._length

        # Load Low Dimensional Data #
        keys_to_ignore = [*keys_to_ignore.copy(), "videos"]
        timestep = load_hdf5_to_dict(self._hdf5_file, self._index, keys_to_ignore=keys_to_ignore)

        # Load High Dimensional Data #
        if self._read_images:
            camera_obs = self._uncompress_images()
            timestep["observation"]["image"] = camera_obs

        # Increment Read Index #
        self._index += 1

        # Return Timestep #
        return timestep

    def _uncompress_images(self):
        # WARNING: THIS FUNCTION HAS NOT BEEN TESTED. UNDEFINED BEHAVIOR FOR FAILED READING. #
        video_folder = self._hdf5_file["observations/videos"]
        camera_obs = {}

        for video_id in video_folder:
            # Create Video Reader If One Hasn't Been Made #
            if video_id not in self._video_readers:
                serialized_video = video_folder[video_id]
                filename = create_video_file(byte_contents=serialized_video)
                self._video_readers[video_id] = imageio.get_reader(filename)

            # Read Next Frame #
            camera_obs[video_id] = yield self._video_readers[video_id]
            # Future Note: Could Make Thread For Each Image Reader

        # Return Camera Observation #
        return camera_obs

    def close(self):
        self._hdf5_file.close()



##############################################################

def run_trajectory(
    env,
    controller,
    horizon=None,
    save_filepath=None,
    metadata=None,
    wait_for_controller=False,
    obs_pointer=None,
    recording_folderpath=False,
    randomize_reset=False,
    reset_robot=True,
    post_process=False,
):
    if post_process:
        assert save_filepath is not None, "Must save data to post process"

    controller.reset_state()
    env.set_camera_trajectory_mode()

    # Prepare Data Writers If Necesary #
    if save_filepath:
        traj_writer = TrajectoryWriter(save_filepath, metadata=metadata, post_process=post_process)
    if recording_folderpath:
        env.start_camera_recording(recording_folderpath)

    # Prepare For Trajectory #
    num_steps = 0
    if reset_robot:
        env.reset()

    # Begin! #
    while True:
        # Collect Miscellaneous Info #
        controller_info = controller.get_info()
        skip_action = wait_for_controller and (not controller_info["movement_enabled"])
        control_timestamps = {"step_start": time_ms()}

        # Get Observation #
        obs = env.get_observation()
        if obs_pointer is not None:
            obs_pointer.update(obs)
        obs["controller_info"] = controller_info
        obs["timestamp"]["skip_action"] = skip_action

        # Get Action #
        control_timestamps["policy_start"] = time_ms()
        action, controller_action_info = controller.forward(obs)

        # Regularize Control Frequency #
        control_timestamps["sleep_start"] = time_ms()
        comp_time = time_ms() - control_timestamps["step_start"]
        sleep_left = (1 / env.get_control_hz()) - (comp_time / 1000)
        if sleep_left > 0:
            time.sleep(sleep_left)

        # Step Environment #
        control_timestamps["control_start"] = time_ms()
        if skip_action:
            action_info = env.create_action_dict(np.zeros_like(action))
        else:
            action_info = env.step(action)
        action_info.update(controller_action_info)

        # Save Data #
        control_timestamps["step_end"] = time_ms()
        obs["timestamp"]["control"] = control_timestamps
        timestep = {"observation": obs, "action": action_info}
        if save_filepath:
            traj_writer.write_timestep(timestep)

        # Check Termination #
        num_steps += 1
        if horizon is not None:
            end_traj = horizon == num_steps
        else:
            end_traj = controller_info["success"] or controller_info["failure"]

        # Close Files And Return #
        if end_traj:
            if recording_folderpath:
                env.stop_camera_recording()
            if save_filepath:
                traj_writer.close(metadata=controller_info)
            return controller_info


def load_trajectory(
    filepath=None,
    read_cameras=True,
    recording_folderpath=None,
    camera_kwargs={},
    remove_skipped_steps=False,
    num_samples_per_traj=None,
    num_samples_per_traj_coeff=1.5,
):
    read_hdf5_images = read_cameras and (recording_folderpath is None)
    read_recording_folderpath = read_cameras and (recording_folderpath is not None)

    traj_reader = TrajectoryReader(filepath, read_images=read_hdf5_images)
    if read_recording_folderpath:
        camera_reader = RecordedMultiCameraWrapper(recording_folderpath, camera_kwargs)

    horizon = traj_reader.length()
    timestep_list = []

    # Choose Timesteps To Save #
    if num_samples_per_traj:
        num_to_save = num_samples_per_traj
        if remove_skipped_steps:
            num_to_save = int(num_to_save * num_samples_per_traj_coeff)
        max_size = min(num_to_save, horizon)
        indices_to_save = np.sort(np.random.choice(horizon, size=max_size, replace=False))
    else:
        indices_to_save = np.arange(horizon)

    # Iterate Over Trajectory #
    for i in indices_to_save:
        # Get HDF5 Data #
        timestep = traj_reader.read_timestep(index=i)

        # If Applicable, Get Recorded Data #
        if read_recording_folderpath:
            timestamp_dict = timestep["observation"]["timestamp"]["cameras"]
            camera_type_dict = {
                k: camera_type_to_string_dict[v] for k, v in timestep["observation"]["camera_type"].items()
            }
            camera_obs = camera_reader.read_cameras(
                index=i, camera_type_dict=camera_type_dict, timestamp_dict=timestamp_dict
            )
            camera_failed = camera_obs is None

            # Add Data To Timestep If Successful #
            if camera_failed:
                break
            else:
                timestep["observation"].update(camera_obs)

        # Filter Steps #
        step_skipped = timestep["observation"]["timestamp"]["skip_action"]
        delete_skipped_step = step_skipped and remove_skipped_steps

        # Save Filtered Timesteps #
        if delete_skipped_step:
            del timestep
        else:
            timestep_list.append(timestep)

    # Remove Extra Transitions #
    timestep_list = np.array(timestep_list)
    if (num_samples_per_traj is not None) and (len(timestep_list) > num_samples_per_traj):
        ind_to_keep = np.random.choice(len(timestep_list), size=num_samples_per_traj, replace=False)
        timestep_list = timestep_list[ind_to_keep]

    # Close Readers #
    traj_reader.close()
    if read_recording_folderpath:
        camera_reader.disable_cameras()

    # Return Data #
    return timestep_list


def visualize_timestep(timestep, max_width=1000, max_height=500, aspect_ratio=1.5, pause_time=15):
    # Process Image Data #
    obs = timestep["observation"]
    if "image" in obs:
        img_obs = obs["image"]
    elif "image" in obs["camera"]:
        img_obs = obs["camera"]["image"]
    else:
        raise ValueError

    camera_ids = sorted(img_obs.keys())
    sorted_image_list = []
    for cam_id in camera_ids:
        data = img_obs[cam_id]
        if type(data) == list:
            sorted_image_list.extend(data)
        else:
            sorted_image_list.append(data)

    # Get Ideal Number Of Rows #
    num_images = len(sorted_image_list)
    max_num_rows = int(num_images**0.5)
    for num_rows in range(max_num_rows, 0, -1):
        num_cols = num_images // num_rows
        if num_images % num_rows == 0:
            break

    # Get Per Image Shape #
    max_img_width, max_img_height = max_width // num_cols, max_height // num_rows
    if max_img_width > aspect_ratio * max_img_height:
        img_width, img_height = max_img_width, int(max_img_width / aspect_ratio)
    else:
        img_width, img_height = int(max_img_height * aspect_ratio), max_img_height

    # Fill Out Image Grid #
    img_grid = [[] for i in range(num_rows)]

    for i in range(len(sorted_image_list)):
        img = Image.fromarray(sorted_image_list[i])
        resized_img = img.resize((img_width, img_height), Image.Resampling.LANCZOS)
        img_grid[i % num_rows].append(np.array(resized_img))

    # Combine Images #
    for i in range(num_rows):
        img_grid[i] = np.hstack(img_grid[i])
    img_grid = np.vstack(img_grid)

    # Visualize Frame #
    cv2.imshow("Image Feed", img_grid)
    cv2.waitKey(pause_time)


def visualize_trajectory(
    filepath,
    recording_folderpath=None,
    remove_skipped_steps=False,
    camera_kwargs={},
    max_width=1000,
    max_height=500,
    aspect_ratio=1.5,
):
    traj_reader = TrajectoryReader(filepath, read_images=True)
    if recording_folderpath:
        if camera_kwargs is {}:
            camera_kwargs = defaultdict(lambda: {"image": True})
        camera_reader = RecordedMultiCameraWrapper(recording_folderpath, camera_kwargs)

    horizon = traj_reader.length()
    camera_failed = False

    for i in range(horizon):
        # Get HDF5 Data #
        timestep = traj_reader.read_timestep()

        # If Applicable, Get Recorded Data #
        if recording_folderpath:
            timestamp_dict = timestep["observation"]["timestamp"]["cameras"]
            camera_type_dict = {
                k: camera_type_to_string_dict[v] for k, v in timestep["observation"]["camera_type"].items()
            }
            camera_obs = camera_reader.read_cameras(
                index=i, camera_type_dict=camera_type_dict, timestamp_dict=timestamp_dict
            )
            camera_failed = camera_obs is None

            # Add Data To Timestep #
            if not camera_failed:
                timestep["observation"].update(camera_obs)

        # Filter Steps #
        step_skipped = timestep["observation"]["timestamp"]["skip_action"]
        delete_skipped_step = step_skipped and remove_skipped_steps
        delete_step = delete_skipped_step or camera_failed
        if delete_step:
            continue

        # Get Image Info #
        assert "image" in timestep["observation"]
        img_obs = timestep["observation"]["image"]
        camera_ids = list(img_obs.keys())
        len(camera_ids)
        camera_ids.sort()

        # Visualize Timestep #
        visualize_timestep(
            timestep, max_width=max_width, max_height=max_height, aspect_ratio=aspect_ratio, pause_time=15
        )

    # Close Readers #
    traj_reader.close()
    if recording_folderpath:
        camera_reader.disable_cameras()