from enum import Flag, Enum
import simplejson
import struct
import math
import io
from pathlib import Path
from dataclasses import dataclass
import altamus_py.mavlink as mavlink
import numpy as np
import copy
from pypcd4.pypcd4 import PointCloud, Encoding


class PCDEncoding(Enum):
    ASCII = Encoding.ASCII,
    BINARY = Encoding.BINARY,
    BINARY_COMPRESSED = Encoding.BINARY_COMPRESSED

class PointFlags(Flag):
    HEALTHY = 1
    NO_RESPONSE = 2
    NO_RETURN = 4
    TOO_FAR = 8
    TOO_CLOSE = 16
    OUTLIER = 32


@dataclass
class Preamble:
    version: int = 0
    preamble_stop: int = 0
    notes_start: int = 0
    notes_stop: int = 0
    header_start: int = 0
    header_stop: int = 0
    points_start: int = 0
    points_stop: int = 0

    def to_bytes(self):
        b = bytearray()
        b += struct.pack("<I", self.version)
        b += struct.pack("<I", self.preamble_stop)
        b += struct.pack("<I", self.notes_start)
        b += struct.pack("<I", self.notes_stop)
        b += struct.pack("<I", self.header_start)
        b += struct.pack("<I", self.header_stop)
        b += struct.pack("<I", self.points_start)
        b += struct.pack("<I", self.points_stop)
        return b

    @classmethod
    def from_bytes(cls, data: bytes):
        cursor = 0

        def read_and_increment():
            nonlocal cursor
            res = int.from_bytes(data[cursor:cursor + 4], byteorder='little')
            cursor += 4
            return res

        preamble = Preamble()
        preamble.version = read_and_increment()
        preamble.preamble_stop = read_and_increment()
        preamble.notes_start = read_and_increment()
        preamble.notes_stop = read_and_increment()
        preamble.header_start = read_and_increment()
        preamble.header_stop = read_and_increment()
        preamble.points_start = read_and_increment()
        preamble.points_stop = read_and_increment()
        return preamble


@dataclass
class Header:
    identifier: mavlink.MAVLink_identifier_message
    scan_settings: mavlink.MAVLink_scan_settings_message
    scan_transform: mavlink.MAVLink_scan_transform_message
    lidar_settings: mavlink.MAVLink_lidar_settings_message
    pitch_motor_settings: mavlink.MAVLink_motor_settings_message
    yaw_motor_settings: mavlink.MAVLink_motor_settings_message
    scan_result: mavlink.MAVLink_scan_result_info_message
    orientation: mavlink.MAVLink_orientation_message
    average_power: mavlink.MAVLink_power_information_message
    minimum_power: mavlink.MAVLink_power_information_message
    maximum_power: mavlink.MAVLink_power_information_message

    @classmethod
    def parse_from_bytes(cls, data: bytes, dialect_module):
        parser = dialect_module.MAVLink(", 1, 1")
        parsed_mavlink = parser.parse_buffer(data)

        identifier = [n for n in parsed_mavlink if isinstance(
            n, dialect_module.MAVLink_identifier_message)][0]
        scan_settings = [n for n in parsed_mavlink if isinstance(
            n, dialect_module.MAVLink_scan_settings_message)][0]
        scan_transform = [n for n in parsed_mavlink if isinstance(
            n, dialect_module.MAVLink_scan_transform_message)][0]
        lidar_settings = [n for n in parsed_mavlink if isinstance(
            n, dialect_module.MAVLink_lidar_settings_message)][0]
        pitch_motor_settings = [n for n in parsed_mavlink if isinstance(
            n, dialect_module.MAVLink_motor_settings_message) and n.motor == dialect_module.EOS_COMPONENT_PITCH_MOTOR][0]
        yaw_motor_settings = [n for n in parsed_mavlink if isinstance(
            n, dialect_module.MAVLink_motor_settings_message) and n.motor == dialect_module.EOS_COMPONENT_YAW_MOTOR][0]
        scan_result = [n for n in parsed_mavlink if isinstance(
            n, dialect_module.MAVLink_scan_result_info_message)][0]
        orientation = [n for n in parsed_mavlink if isinstance(
            n, dialect_module.MAVLink_orientation_message)][0]
        average_power = [n for n in parsed_mavlink if isinstance(
            n, dialect_module.MAVLink_power_information_message) and n.type == dialect_module.POWER_INFORMATION_TYPE_AVERAGE][0]
        minimum_power = [n for n in parsed_mavlink if isinstance(
            n, dialect_module.MAVLink_power_information_message) and n.type == dialect_module.POWER_INFORMATION_TYPE_MINIMUM][0]
        maximum_power = [n for n in parsed_mavlink if isinstance(
            n, dialect_module.MAVLink_power_information_message) and n.type == dialect_module.POWER_INFORMATION_TYPE_MAXIMUM][0]

        header = Header(identifier=identifier, scan_settings=scan_settings, scan_transform=scan_transform, lidar_settings=lidar_settings, pitch_motor_settings=pitch_motor_settings,
                        yaw_motor_settings=yaw_motor_settings, scan_result=scan_result, orientation=orientation, average_power=average_power, minimum_power=minimum_power, maximum_power=maximum_power)
        return header

    @classmethod
    def from_dict(cls, dict: dict):
        foo = dict.get("header")
        if foo is None:
            raise ValueError("no match for entry 'header' in the parsed dict")

        # Get the message data from the dict
        parsed_identifier = foo.get("IDENTIFIER")
        parsed_scan_settings = foo.get("SCAN_SETTINGS")
        parsed_scan_transform = foo.get("SCAN_TRANSFORM")
        parsed_lidar_settings = foo.get("LIDAR_SETTINGS")
        parsed_pitch_motor_settings = foo.get("EOS_COMPONENT_PITCH_MOTOR")
        parsed_yaw_motor_settings = foo.get("EOS_COMPONENT_YAW_MOTOR")
        parsed_scan_result = foo.get("SCAN_RESULT_INFO")
        parsed_orientation = foo.get("ORIENTATION")
        parsed_average_power = foo.get("POWER_INFORMATION_TYPE_AVERAGE")
        parsed_minimum_power = foo.get("POWER_INFORMATION_TYPE_MINIMUM")
        parsed_maximum_power = foo.get("POWER_INFORMATION_TYPE_MAXIMUM")

        # go through each one and create a mavlink object from the data
        identifier = mavlink.MAVLink_identifier_message(
            fw_version=parsed_identifier.get("fw_version"),
            particle_id=parsed_identifier.get("particle_id").encode(),
            device_id=parsed_identifier.get("device_id").encode(),
            name=parsed_identifier.get("name").encode(),
            local_ip=parsed_identifier.get("local_ip"),
            mac=parsed_identifier.get("mac"))
        scan_settings = mavlink.MAVLink_scan_settings_message(
            yaw_start=parsed_scan_settings.get("yaw_start"),
            yaw_stop=parsed_scan_settings.get("yaw_stop"),
            pitch_start=parsed_scan_settings.get("pitch_start"),
            pitch_stop=parsed_scan_settings.get("pitch_stop"),
            pitch_rest_angle=parsed_scan_settings.get("pitch_rest_angle"),
            point_spacing=parsed_scan_settings.get("point_spacing"),
            scan_speed=parsed_scan_settings.get("scan_speed"),
            scan_stop_reasons=parsed_scan_settings.get("scan_stop_reasons"))
        scan_transform = mavlink.MAVLink_scan_transform_message(
            roll_offset=parsed_scan_transform.get("roll_offset"),
            pitch_offset=parsed_scan_transform.get("pitch_offset"),
            pitch_scale=parsed_scan_transform.get("pitch_scale"),
            yaw_scale=parsed_scan_transform.get("yaw_scale"),
            range_scale=parsed_scan_transform.get("range_scale"),
            max_range=parsed_scan_transform.get("max_range"))
        lidar_settings = mavlink.MAVLink_lidar_settings_message(
            update_rate=parsed_lidar_settings.get("update_rate"),
            fog_mode_enable=parsed_lidar_settings.get("fog_mode_enable"),
            output_disabled_at_boot=parsed_lidar_settings.get(
                "output_disabled_at_boot"),
            firmware_version=parsed_lidar_settings.get("firmware_version").encode())
        pitch_motor_settings = mavlink.MAVLink_motor_settings_message(
            motor=parsed_pitch_motor_settings.get("motor"),
            current=parsed_pitch_motor_settings.get("current"),
            microsteps=parsed_pitch_motor_settings.get("microsteps"),
            gearing_ratio=parsed_pitch_motor_settings.get("gearing_ratio"),
            spread_cycle=parsed_pitch_motor_settings.get("spread_cycle"),
            pwm_autograd=parsed_pitch_motor_settings.get("pwm_autograd"),
            pwm_autoscale=parsed_pitch_motor_settings.get("pwm_autoscale"),
            home_offset_steps=parsed_pitch_motor_settings.get(
                "home_offset_steps"),
            enforce_minimum_steps=parsed_pitch_motor_settings.get(
                "enforce_minimum_steps"),
            steps_to_next_index=parsed_pitch_motor_settings.get(
                "steps_to_next_index"),
            usteps_rate=parsed_pitch_motor_settings.get("usteps_rate"),
            ustep_angle=parsed_pitch_motor_settings.get("ustep_angle"))
        yaw_motor_settings = mavlink.MAVLink_motor_settings_message(
            motor=parsed_yaw_motor_settings.get("motor"),
            current=parsed_yaw_motor_settings.get("current"),
            microsteps=parsed_yaw_motor_settings.get("microsteps"),
            gearing_ratio=parsed_yaw_motor_settings.get("gearing_ratio"),
            spread_cycle=parsed_yaw_motor_settings.get("spread_cycle"),
            pwm_autograd=parsed_yaw_motor_settings.get("pwm_autograd"),
            pwm_autoscale=parsed_yaw_motor_settings.get("pwm_autoscale"),
            home_offset_steps=parsed_yaw_motor_settings.get(
                "home_offset_steps"),
            enforce_minimum_steps=parsed_yaw_motor_settings.get(
                "enforce_minimum_steps"),
            steps_to_next_index=parsed_yaw_motor_settings.get(
                "steps_to_next_index"),
            usteps_rate=parsed_yaw_motor_settings.get("usteps_rate"),
            ustep_angle=parsed_yaw_motor_settings.get("ustep_angle"))
        scan_result = mavlink.MAVLink_scan_result_info_message(
            type=parsed_scan_result.get("type"),
            num_points=parsed_scan_result.get("num_points"),
            file_size_bytes=parsed_scan_result.get("file_size_bytes"),
            start_time_unix=parsed_scan_result.get("start_time_unix"),
            end_time_unix=parsed_scan_result.get("end_time_unix"),
            scan_duration=parsed_scan_result.get("scan_duration"),
            scan_stop_reason=parsed_scan_result.get("scan_stop_reason"),
            scan_start_reason=parsed_scan_result.get("scan_start_reason"))
        orientation = mavlink.MAVLink_orientation_message(
            roll=parsed_orientation.get("roll"),
            pitch=parsed_orientation.get("pitch"),
            temp=parsed_orientation.get("temp"),
            xmag=parsed_orientation.get("xmag"),
            ymag=parsed_orientation.get("ymag"),
            zmag=parsed_orientation.get("zmag"),
            heading=parsed_orientation.get("heading"),
            lat=parsed_orientation.get("lat"),
            lon=parsed_orientation.get("lon"),
            h_acc=parsed_orientation.get("h_acc"),
            v_acc=parsed_orientation.get("v_acc"),
            alt=parsed_orientation.get("alt"))
        average_power = mavlink.MAVLink_power_information_message(
            type=parsed_average_power.get("type"),
            current=parsed_average_power.get("current"),
            voltage=parsed_average_power.get("voltage"),
            power=parsed_average_power.get("power"),
            energy_consumed=parsed_average_power.get("energy_consumed"),)
        minimum_power = mavlink.MAVLink_power_information_message(
            type=parsed_minimum_power.get("type"),
            current=parsed_minimum_power.get("current"),
            voltage=parsed_minimum_power.get("voltage"),
            power=parsed_minimum_power.get("power"),
            energy_consumed=parsed_minimum_power.get("energy_consumed"),)
        maximum_power = mavlink.MAVLink_power_information_message(
            type=parsed_maximum_power.get("type"),
            current=parsed_maximum_power.get("current"),
            voltage=parsed_maximum_power.get("voltage"),
            power=parsed_maximum_power.get("power"),
            energy_consumed=parsed_maximum_power.get("energy_consumed"),)

        # return the mavlink based header
        return Header(identifier=identifier, scan_settings=scan_settings, scan_transform=scan_transform, lidar_settings=lidar_settings, pitch_motor_settings=pitch_motor_settings,
                      yaw_motor_settings=yaw_motor_settings, scan_result=scan_result, orientation=orientation, average_power=average_power, minimum_power=minimum_power, maximum_power=maximum_power)

    def to_bytes(self) -> bytes:
        b = bytearray()
        if self.identifier is not None:
            b += self.identifier.pack(mavlink.MAVLink("", 1, 1))
        if self.scan_settings is not None:
            b += self.scan_settings.pack(mavlink.MAVLink("", 1, 1))
        if self.scan_transform is not None:
            b += self.scan_transform.pack(mavlink.MAVLink("", 1, 1))
        if self.lidar_settings is not None:
            b += self.lidar_settings.pack(mavlink.MAVLink("", 1, 1))
        if self.pitch_motor_settings is not None:
            b += self.pitch_motor_settings.pack(mavlink.MAVLink("", 1, 1))
        if self.yaw_motor_settings is not None:
            b += self.yaw_motor_settings.pack(mavlink.MAVLink("", 1, 1))
        if self.scan_result is not None:
            b += self.scan_result.pack(mavlink.MAVLink("", 1, 1))
        if self.orientation is not None:
            b += self.orientation.pack(mavlink.MAVLink("", 1, 1))
        if self.average_power is not None:
            b += self.average_power.pack(mavlink.MAVLink("", 1, 1))
        if self.minimum_power is not None:
            b += self.minimum_power.pack(mavlink.MAVLink("", 1, 1))
        if self.maximum_power is not None:
            b += self.maximum_power.pack(mavlink.MAVLink("", 1, 1))

        return b

    def to_json(self) -> str:
        return simplejson.dumps(self.to_dict_annotated(), ignore_nan=True)

    def to_dict_annotated(self) -> dict:
        j = {}
        for field in self.__dataclass_fields__:
            value = getattr(self, field)
            if value is None:
                continue
            chapter = value.to_dict()
            key_name = chapter['mavpackettype']
            # there are multiple MOTOR_SETTINGS messages, override the dict key here based on the 'motor' field
            if key_name == 'MOTOR_SETTINGS':
                if chapter['motor'] == mavlink.EOS_COMPONENT_PITCH_MOTOR:
                    key_name = "EOS_COMPONENT_PITCH_MOTOR"
                elif chapter['motor'] == mavlink.EOS_COMPONENT_YAW_MOTOR:
                    key_name = "EOS_COMPONENT_YAW_MOTOR"
            # there are also multiple POWER_INFORMATION
            if key_name == 'POWER_INFORMATION':
                if chapter['type'] == mavlink.POWER_INFORMATION_TYPE_AVERAGE:
                    key_name = "POWER_INFORMATION_TYPE_AVERAGE"
                elif chapter['type'] == mavlink.POWER_INFORMATION_TYPE_MINIMUM:
                    key_name = "POWER_INFORMATION_TYPE_MINIMUM"
                elif chapter['type'] == mavlink.POWER_INFORMATION_TYPE_MAXIMUM:
                    key_name = "POWER_INFORMATION_TYPE_MAXIMUM"
            j[key_name] = chapter

        return j

@dataclass
class EosV2BinFile:

    file_path: Path | None = None
    data: bytes | None = None

    @staticmethod
    def from_file(file_path: Path) -> "EosV2BinFile":
        """
        Create a EosV2BinFile object from a binary file
        
        :param file_path: Path to the EOS binary scan.bin
        :type file_path: Path
        :return: EosV2BinFile
        :rtype: EosV2BinFile
        """
        data: bytes
        with open(file_path, "rb") as file:
            data = file.read()
        print(
            f"Read in {len(data)} bytes from binfile {file_path.as_posix()}")
        return EosV2BinFile(file_path=file_path, data=data)

    @staticmethod
    def from_bytes(data: bytes) -> "EosV2BinFile":
        """
        Generate EosV2BinFile object from bytes, usually as a result from a network request
        
        :param data: bytes from an EOS binary file
        :type data: bytes
        :return: EosV2BinFile
        :rtype: EosV2BinFile
        """
        return EosV2BinFile(file_path=None, data=data)

    @property
    def preamble(self) -> Preamble:
        """
        Return the Preamble of the binary file. Contains start/stop locations of the blocks of data making up the binary file
        
        :return: Binary File Preamble
        :rtype: Preamble
        """
        if self.data is None:
            return Preamble()
        return Preamble.from_bytes(self.data)

    @property
    def header_bytes(self) -> bytes:
        """
        Get the bytes making up the Header portion of the binary file
        
        :return: Header bytes
        :rtype: bytes
        """
        if self.preamble.header_start is None or self.preamble.header_stop is None or self.data is None:
            return bytes(0)
        return self.data[self.preamble.header_start:self.preamble.header_stop + 1]

    @property
    def notes_bytes(self) -> bytes:
        """
        Get the bytes making up the Notes portion of the binary file
        
        :return: Notes bytes
        :rtype: bytes
        """
        if self.preamble.notes_start is None or self.preamble.notes_stop is None or self.data is None:
            return bytes(0)
        return self.data[self.preamble.notes_start:self.preamble.notes_stop + 1]

    @property
    def points_bytes(self) -> bytes:
        """
        Get the bytes making up the Points portion of the binary file
        
        :return: Points bytes
        :rtype: bytes
        """
        if self.preamble.points_start is None or self.preamble.points_stop is None or self.data is None:
            return bytes(0)

        # slice out all data after the points_start attribute
        expected_points_data_length = self.preamble.points_stop - self.preamble.points_start
        points_data = self.data[self.preamble.points_start:]
        if expected_points_data_length > len(points_data):
            print(
                f"Expected {expected_points_data_length} bytes of point data, only got {len(points_data)}, returning what is available")
        return points_data


class EOSV2Scan:
    def __init__(self):
        self.bin_file: EosV2BinFile | None
        self.header: Header
        self.notes = ""
        self.polar_points: np.ndarray = np.empty(
            (0, 4))  # distance, pitch, yaw, intensity

    @property
    def missing_points_count(self) -> int:
        """
        Number of missing points from the scan

        If scan is user cancelled, this returns 0, since we can't trust the estimated count anymore.
        """
        return max(0, self.expected_points_count - self.points_count)

    @property
    def expected_points_count(self) -> int:
        """
        Number of expected points the scan should have based on the scan settings
        
        returns 0 if no value can be calculated

        If the scan was canceled, return the actual point count.

        :return: Number of points expected
        :rtype: int
        """
        if self.header.scan_result.scan_stop_reason == mavlink.SCAN_STOP_REASON_USER_CANCELED:
            return self.points_count
        return self.calculated_expected_points_count

    @property
    def calculated_expected_points_count(self) -> int:
        """
        Number of points that should have been captured, based on the settings that were used when the scan was started

        :return: Calculated number of points
        :rtype: int
        """
        settings = self.header.scan_settings

        pitch_range = settings.pitch_stop - settings.pitch_start
        yaw_range = settings.yaw_stop - settings.yaw_start
        points_per_pitch_rotation = pitch_range / settings.point_spacing
        number_of_rotations = yaw_range / settings.point_spacing
        estimated_points = points_per_pitch_rotation * number_of_rotations
        return int(estimated_points)

    @property
    def unhealthy_points_count(self) -> int:
        """
        Number of unhealthy points in the scan

        defined as any point that has any PointFlags other than HEALTHY
    
        
        :param self: Description
        :return: unhealthy points 
        :rtype: int
        """
        pts = np.where(self.cartesian_points[:, 3] != PointFlags.HEALTHY.value)
        return len(pts[0])

    @property
    def healthy_points_count(self) -> int:
        """
        Number of healthy points in the scan.
        
        :return: healthy points
        :rtype: int
        """
        pts = np.where(self.cartesian_points[:, 3] == PointFlags.HEALTHY.value)
        return len(pts[0])
    
    @property
    def no_response_points_count(self) -> int:
        """
        Number of No Response points in the scan. No Response points are bad as they indicate the lidar module didn't return any data

        :return: Number of No Response Points
        :rtype: int
        """
        pts = np.where(
            self.cartesian_points[:, 3] == PointFlags.NO_RESPONSE.value)
        return len(pts[0])

    @property
    def no_return_points_count(self) -> int:
        """
        Number of No Return points. No Return points are normal in conditions like pointing at the sky or at water surface. Abnormally high amounts may indicate dirty lens
        
        :return: Number of No Return Points
        :rtype: int
        """
        pts = np.where(
            self.cartesian_points[:, 3] == PointFlags.NO_RETURN.value)
        return len(pts[0])

    @property
    def too_close_points_count(self) -> int:
        pts = np.where(
            self.cartesian_points[:, 3] == PointFlags.TOO_CLOSE.value)
        return len(pts[0])

    @property
    def too_far_points_count(self) -> int:
        pts = np.where(
            self.cartesian_points[:, 3] == PointFlags.TOO_FAR.value)
        return len(pts[0])

    @property
    def points_count(self) -> int:
        return len(self.polar_points)

    @property
    def combined_points(self) -> np.ndarray:
        """
        Returns combined polar and cartesian points as numpy array. 
        Columns are: X, Y, Z, Flags, Distance, Pitch, Yaw, Intensity
        
        :return: Combined points
        :rtype: ndarray[shape(number_of_points, 8), dtype[Float32]]
        """
        return np.hstack((self.cartesian_points, self.polar_points))

    @property
    def cartesian_points(self) -> np.ndarray:
        """
        Returns derived cartesian points. Takes into account the scan transform in the header and automatically updates if the transform changes.
        Columns are: X, Y, Z, Flags
        
        :return: Description
        :rtype: ndarray[shape(number_of_points, 4), dtype[Float32]]
        """
        def pol2cart(pitch, yaw, distance_cm):
            transform = copy.deepcopy(self.header.scan_transform)
            if transform is None:
                print("no transform in the header, using a default value")
                transform = mavlink.MAVLink_scan_transform_message(
                    roll_offset=0,
                    pitch_offset=0,
                    pitch_scale=1.0,
                    yaw_scale=1.0,
                    range_scale=1.0,
                    max_range=18000)

            flags = np.full(shape=pitch.shape,
                            fill_value=PointFlags.HEALTHY.value)
            # TODO order is too important here, if we don't do it exactly like this they'll overwrite each other
            flags[distance_cm > transform.max_range] = PointFlags.TOO_FAR.value
            flags[distance_cm < 100] = PointFlags.TOO_CLOSE.value
            flags[distance_cm == 65535] = PointFlags.NO_RESPONSE.value
            flags[distance_cm == 0] = PointFlags.NO_RETURN.value

            error_mask = flags != PointFlags.HEALTHY.value
            radius_meters = distance_cm / 100.0
            radius_meters[error_mask] = 0.5
            pitch_radians = pitch / 10000
            yaw_radians = yaw / 10000
            roll_offset_radians = np.radians(
                transform.roll_offset)
            pitch_offset_radians = np.radians(
                transform.pitch_offset)

            pitch_adjusted_radians = pitch_radians + pitch_offset_radians

            x = np.cos(yaw_radians) * np.sin(pitch_adjusted_radians) * np.cos(
                roll_offset_radians) + np.sin(yaw_radians) * np.sin(roll_offset_radians)
            y = np.sin(yaw_radians) * np.sin(pitch_adjusted_radians) * np.cos(
                roll_offset_radians) - np.cos(yaw_radians) * np.sin(roll_offset_radians)
            z = np.cos(pitch_adjusted_radians) * np.cos(roll_offset_radians)

            x = x * radius_meters
            y = y * radius_meters
            z = z * radius_meters
            return np.stack((x, y, z, flags), axis=1)

        pitch = self.polar_points[:, 1]
        yaw = self.polar_points[:, 2]
        distance = self.polar_points[:, 0]
        return pol2cart(pitch, yaw, distance)

    def get_filtered_points(self, point_types_to_get: list[PointFlags]):
        values = np.array([member.value for member in point_types_to_get])
        flags = self.combined_points[:, 3]
        mask = np.isin(flags, values)
        filtered = self.combined_points[mask]
        return filtered

    def apply_new_transform_to_scan(self, transform: mavlink.MAVLink_scan_transform_message):
        self.header.scan_transform = transform

    def _parse_polar_points_from_bytes(self, data: bytes):

        # parse raw bytes into polar points
        numpoints = int(len(data) / 8)
        foo = np.frombuffer(data, dtype=np.uint16)

        # Reshape to correct shape for easier column based parsing later
        self.polar_points = foo.reshape((numpoints, 4))


    @property
    def yaw_overlap_points(self) -> tuple[float, np.ndarray | None, np.ndarray | None]:
        """
        Returns the points from the scan that overlap each other. If overlap is 0, points entries are None
        format is [overlap in degrees, beginning_points (low yaw angle), ending_points (high yaw angle)]

        :return: Description
        :rtype: tuple[overlap_degrees: float, beginning_points: np.ndarray | None, ending_points: np.ndarray | None]
        """
        # creates a boolean index array
        overlap_bool = self.combined_points[:, 6] > (np.pi * 10000)
        # creates new array based on above boolean mask
        overlap_array = self.combined_points[overlap_bool]

        if len(overlap_array) == 0:
            return (0.0, None, None)

        # get min and max yaw values. Subtract them to get the yaw range. This is where we'll select from to get the "primary" points
        overlap_min_yaw = overlap_array[0][6]
        overlap_max_yaw = overlap_array[-1][6]
        range = np.degrees((overlap_max_yaw - overlap_min_yaw) / 10000)
        # print(f"Overlap Range of {range}")

        # Get all points who's yaw is under the max value of the overlap minus 180 degrees (Pi)
        primary_bool = self.combined_points[:, 6] < overlap_max_yaw - (np.pi * 10000)
        primary_array = self.combined_points[primary_bool]
        return range, primary_array, overlap_array

    def save_annotated_pcd_to_file(self, filename: Path, include_error_points: bool = True, encoding: PCDEncoding = PCDEncoding.BINARY_COMPRESSED):
        pcd = self.as_pcd(include_error_points)

        # Write the header as a comment to the top line of the PCD
        v_file = io.BytesIO()
        header = {"header": self.header.to_dict_annotated(),
                  "notes": self.notes}
        v_file.write(f"#{simplejson.dumps(header, ignore_nan=True)}".encode())
        v_file.write(f"\n\r".encode())

        # write the pcd file
        enc: Encoding
        match encoding:
            case PCDEncoding.BINARY:
                enc = Encoding.BINARY
            case PCDEncoding.ASCII:
                enc = Encoding.ASCII
            case PCDEncoding.BINARY_COMPRESSED:
                enc = Encoding.BINARY_COMPRESSED
        pcd.save(v_file, encoding=enc)

        # save to filesystem
        with open(filename, mode="wb") as pcd_file:
            pcd_file.write(v_file.getvalue())

    def as_pcd(self, include_error_points: bool = True) -> PointCloud:
        if include_error_points:
            points = self.combined_points
        else:
            points = self.get_filtered_points([PointFlags.HEALTHY])

        fields = ("x", "y", "z", "flags", "distance",
                  "pitch", "yaw", "intensity")
        types = (np.float32, np.float32, np.float32, np.uint8,
                 np.int16, np.uint16, np.uint16, np.uint16,)

        return PointCloud.from_points(points, fields, types)

    @classmethod
    def _extract_header_json_from_pcd_file(cls, path_to_file: Path) -> dict:
        header_dict = {}
        with open(path_to_file, mode='rb') as file:
            for line in file:
                line = line.decode()
                if line.startswith("#") and not header_dict:
                    line = line.replace("#", "")
                    try:
                        json = simplejson.loads(line)
                        header_dict = json
                    except simplejson.JSONDecodeError:
                        print("comment line not valid json, skipping")
                else:
                    break
        return header_dict

    @classmethod
    def _find_header_in_pcd_file(cls, path_to_file: Path) -> Header:
        return Header.from_dict(EOSV2Scan._extract_header_json_from_pcd_file(path_to_file))

    @classmethod
    def _find_notes_in_pcd_file(cls, path_to_file: Path) -> str:
        notes = EOSV2Scan._extract_header_json_from_pcd_file(
            path_to_file).get("notes")
        if notes is None:
            notes = ""
        return notes

    @classmethod
    def _generate_preamble(cls, header: Header, notes: str, points: bytes) -> Preamble:
        # FIXME something is borked in the spacing of the preamble between this parsed one and the scan. Some off by one errors with byte slicing. figure out later
        preamble_stop = 31
        notes_start = 32
        notes_stop = notes_start + len(notes) - 1
        header_start = notes_stop + 1
        header_stop = header_start + len(header.to_bytes()) - 1
        points_start = header_stop + 1
        points_stop = points_start + len(points)
        return Preamble(version=1, preamble_stop=preamble_stop, notes_start=notes_start, notes_stop=notes_stop, header_start=header_start, header_stop=header_stop, points_start=points_start, points_stop=points_stop)

    @classmethod
    def _pcd_to_binfile(cls, path_to_file: Path) -> EosV2BinFile:
        header = EOSV2Scan._find_header_in_pcd_file(path_to_file)
        notes = EOSV2Scan._find_notes_in_pcd_file(path_to_file)
        points = EOSV2Scan._create_points_bytes_from_pcd(
            PointCloud.from_path(path_to_file))
        preamble = EOSV2Scan._generate_preamble(header, notes, points)
        b = bytearray()
        b += preamble.to_bytes()
        b += notes.encode()
        b += header.to_bytes()
        b += points

        binfile = EosV2BinFile.from_bytes(b)
        return binfile

    @staticmethod
    def _create_points_bytes_from_pcd(point_cloud: PointCloud) -> bytes:
        polar_data = point_cloud.numpy()[:,-4:] # convert the pcd to numpy, and slice out last 4 columns which are dist, pitch, yaw, intensity
        polar_data = polar_data.astype('<u2') # convert to little endian uint16_t
        return polar_data.tobytes()

    @staticmethod
    def from_pcd(path_to_file: Path):
        bin_file = EOSV2Scan._pcd_to_binfile(path_to_file)
        return EOSV2Scan.from_bin_file(bin_file)

    @staticmethod
    def from_bin_file(bin_file: EosV2BinFile):
        scan = EOSV2Scan()
        scan.bin_file = bin_file
        scan._parse_binfile_data()
        return scan

    @staticmethod
    def from_path(path_to_file: Path):
        scan = EOSV2Scan()
        scan.bin_file = EosV2BinFile.from_file(path_to_file)
        scan._parse_binfile_data()
        return scan

    def _parse_binfile_data(self):
        if self.bin_file is None:
            return

        preamble = self.bin_file.preamble

        # If preamble has header start/stop, parse the header
        if (preamble.header_start is not None and preamble.header_stop is not None):
            self.header = Header.parse_from_bytes(
                self.bin_file.header_bytes, mavlink)

        # If preamble has notes start/stop, parse them
        if (preamble.notes_start is not None and preamble.notes_stop is not None):
            self.notes = self.bin_file.notes_bytes.decode().rstrip('\x00')

        # if preamble has points start, parse from there to the end of the file
        if (preamble.points_start is not None):
            self._parse_polar_points_from_bytes(self.bin_file.points_bytes)

    def polar_points_to_json(self) -> str:
        d = self.header.to_dict_annotated()
        points = self.polar_points.copy().astype(np.float32)
        pitch_degrees = np.rad2deg(points[:, 1] / 10000)
        yaw_degrees = np.rad2deg(points[:, 2] / 10000)

        points[:, 1] = pitch_degrees
        points[:, 2] = yaw_degrees

        foo = points.tolist()
        d["NOTES"] = self.notes
        d["POINTS"] = foo
        j = simplejson.dumps(d, ignore_nan=True)
        return j
