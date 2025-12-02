import os
import random
from collections import defaultdict
import glob
from pathlib import Path

from dex_control.camera.zed_camera import gather_zed_cameras
from dex_control.utils.misc.parameters import get_camera_type
from dex_control.camera.mp4_reader import MP4Reader
from dex_control.camera.svo_reader import SVOReader


class MultiCameraWrapper:
    def __init__(self, camera_kwargs=None):
        zed_cameras = gather_zed_cameras()
        self.camera_dict = {cam.serial_number: cam for cam in zed_cameras}
        self.set_camera_kwargs(camera_kwargs)
        self.set_trajectory_mode()
    
    def set_camera_kwargs(self, camera_kwargs):
        if camera_kwargs is None:
            camera_kwargs = {"default": {"depth": False, "pointcloud": False}}
        for cam_id in self.camera_dict.keys():
            cam_type = get_camera_type(cam_id)
            curr_cam_kwargs = camera_kwargs[cam_type] if cam_type in camera_kwargs else camera_kwargs["default"]
            self.camera_dict[cam_id].set_reading_parameters(**curr_cam_kwargs)

    ### Calibration Functions ###
    def get_camera(self, camera_id):
        return self.camera_dict[camera_id]

    def enable_advanced_calibration(self):
        for cam in self.camera_dict.values():
            cam.enable_advanced_calibration()

    def disable_advanced_calibration(self):
        for cam in self.camera_dict.values():
            cam.disable_advanced_calibration()

    def set_calibration_mode(self, cam_id):
        # If High Res Calibration, Only One Can Run #
        close_all = any([cam.high_res_calibration for cam in self.camera_dict.values()])

        if close_all:
            for curr_cam_id in self.camera_dict:
                if curr_cam_id != cam_id:
                    self.camera_dict[curr_cam_id].disable_camera()

        self.camera_dict[cam_id].set_calibration_mode()

    def set_trajectory_mode(self):
        # If High Res Calibration, Close All #
        close_all = any(
            [cam.high_res_calibration and cam.current_mode == "calibration" for cam in self.camera_dict.values()]
        )

        if close_all:
            for cam in self.camera_dict.values():
                cam.disable_camera()

        # Put All Cameras In Trajectory Mode #
        for cam in self.camera_dict.values():
            cam.set_trajectory_mode()

    ### Data Storing Functions ###
    def start_recording(self, recording_folderpath):
        for cam in self.camera_dict.values():
            filepath = os.path.join(recording_folderpath, cam.serial_number + ".svo2")
            cam.start_recording(filepath)

    def stop_recording(self):
        for cam in self.camera_dict.values():
            cam.stop_recording()

    ### Basic Camera Functions ###
    def read_cameras(self):
        full_obs_dict = defaultdict(dict)
        full_timestamp_dict = {}

        # Read Cameras In Randomized Order #
        all_cam_ids = list(self.camera_dict.keys())
        random.shuffle(all_cam_ids)

        for cam_id in all_cam_ids:
            if not self.camera_dict[cam_id].is_running():
                continue
            data_dict, timestamp_dict = self.camera_dict[cam_id].read_camera()

            for key in data_dict:
                full_obs_dict[key].update(data_dict[key])
            full_timestamp_dict.update(timestamp_dict)

        return full_obs_dict, full_timestamp_dict

    def disable_cameras(self):
        for camera in self.camera_dict.values():
            camera.disable_camera()

class RecordedMultiCameraWrapper:
    def __init__(self, recording_folderpath, camera_kwargs={}):
        if camera_kwargs is None:
            camera_kwargs = {"default": {"depth": False, "pointcloud": False}}
        self.camera_kwargs = camera_kwargs

        # Open Camera Readers #
        svo_filepaths = glob.glob(recording_folderpath + "/*.svo2")
        mp4_filepaths = glob.glob(recording_folderpath + "/*.mp4")
        all_filepaths = svo_filepaths  # Only process SVO

        self.camera_dict = {}
        for f in all_filepaths:
            serial_number = Path(f).stem
            cam_type = get_camera_type(serial_number)
            camera_kwargs.get(cam_type, {})

            if f.endswith(".svo2"):
                Reader = SVOReader
            elif f.endswith(".mp4"):
                Reader = MP4Reader
            else:
                raise ValueError

            self.camera_dict[serial_number] = Reader(f, serial_number)

    def read_cameras(self, index=None, timestamp_dict={}):
        full_obs_dict = defaultdict(dict)

        # Read Cameras In Randomized Order #
        all_cam_ids = list(self.camera_dict.keys())
        random.shuffle(all_cam_ids)

        for cam_id in all_cam_ids:
            cam_type = get_camera_type(cam_id)
            curr_cam_kwargs = self.camera_kwargs[cam_type] if cam_type in self.camera_kwargs else self.camera_kwargs["default"]
            self.camera_dict[cam_id].set_reading_parameters(**curr_cam_kwargs)

            timestamp = timestamp_dict.get(cam_id + "_frame_received", None)
            if index is not None:
                self.camera_dict[cam_id].set_frame_index(index)

            data_dict = self.camera_dict[cam_id].read_camera(correct_timestamp=timestamp)

            # Process Returned Data #
            if data_dict is None:
                return None
            for key in data_dict:
                full_obs_dict[key].update(data_dict[key])

        return full_obs_dict

    def disable_cameras(self):
        for camera in self.camera_dict.values():
            camera.disable_camera()