from enum import Flag, Enum
import simplejson
import struct
import math
import io
import os
from pathlib import Path
from dataclasses import dataclass, fields
import altamus_py.mavlink as mavlink
import numpy as np
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
    identifier: mavlink.MAVLink_identifier_message | None = None
    scan_settings: mavlink.MAVLink_scan_settings_message | None = None
    scan_transform: mavlink.MAVLink_scan_transform_message | None = None
    lidar_settings: mavlink.MAVLink_lidar_settings_message | None = None
    pitch_motor_settings: mavlink.MAVLink_motor_settings_message | None = None
    yaw_motor_settings: mavlink.MAVLink_motor_settings_message | None = None
    scan_result: mavlink.MAVLink_scan_result_info_message | None = None
    orientation: mavlink.MAVLink_orientation_message | None = None
    average_power: mavlink.MAVLink_power_information_message | None = None
    minimum_power: mavlink.MAVLink_power_information_message | None = None
    maximum_power: mavlink.MAVLink_power_information_message | None = None

    @classmethod
    def parse_from_bytes(cls, data: bytes, dialect_module):
        header = Header()
        parser = dialect_module.MAVLink(", 1, 1")
        parsed_mavlink = parser.parse_buffer(data)
        if parsed_mavlink is None:
            print("No mavlink parsed, returning empty header")
            return header

        # Loop through the mavlink messages and extract to a "nice" json structure
        for chapter in parsed_mavlink:
            if isinstance(chapter, dialect_module.MAVLink_identifier_message):
                header.identifier = chapter
            elif isinstance(chapter, dialect_module.MAVLink_scan_settings_message):
                header.scan_settings = chapter
            elif isinstance(chapter, dialect_module.MAVLink_scan_transform_message):
                header.scan_transform = chapter
            elif isinstance(chapter, dialect_module.MAVLink_lidar_settings_message):
                header.lidar_settings = chapter
            elif isinstance(chapter, dialect_module.MAVLink_motor_settings_message):
                if (chapter.motor == dialect_module.EOS_COMPONENT_PITCH_MOTOR):
                    header.pitch_motor_settings = chapter
                if (chapter.motor == dialect_module.EOS_COMPONENT_YAW_MOTOR):
                    header.yaw_motor_settings = chapter
            elif isinstance(chapter, dialect_module.MAVLink_scan_result_info_message):
                header.scan_result = chapter
            elif isinstance(chapter, dialect_module.MAVLink_orientation_message):
                header.orientation = chapter
            elif isinstance(chapter, dialect_module.MAVLink_power_information_message):
                if chapter.type == dialect_module.POWER_INFORMATION_TYPE_AVERAGE:
                    header.average_power = chapter
                elif chapter.type == dialect_module.POWER_INFORMATION_TYPE_MINIMUM:
                    header.minimum_power = chapter
                elif chapter.type == dialect_module.POWER_INFORMATION_TYPE_MAXIMUM:
                    header.maximum_power = chapter
        return header

    @classmethod
    def from_dict(cls, dict: dict):
        header = Header()

        foo = dict.get("header")
        if foo is None:
            return header

        # Get the message data from the dict
        identifier = foo.get("IDENTIFIER")
        scan_settings = foo.get("SCAN_SETTINGS")
        scan_transform = foo.get("SCAN_TRANSFORM")
        lidar_settings = foo.get("LIDAR_SETTINGS")
        pitch_motor_settings = foo.get("EOS_COMPONENT_PITCH_MOTOR")
        yaw_motor_settings = foo.get("EOS_COMPONENT_YAW_MOTOR")
        scan_result = foo.get("SCAN_RESULT_INFO")
        orientation = foo.get("ORIENTATION")
        average_power = foo.get("POWER_INFORMATION_TYPE_AVERAGE")
        minimum_power = foo.get("POWER_INFORMATION_TYPE_MINIMUM")
        maximum_power = foo.get("POWER_INFORMATION_TYPE_MAXIMUM")

        # go through each one and create a mavlink object from the data
        header.identifier = mavlink.MAVLink_identifier_message(
            fw_version=identifier.get("fw_version"),
            particle_id=identifier.get("particle_id").encode(),
            device_id=identifier.get("device_id").encode(),
            name=identifier.get("name").encode(),
            local_ip=identifier.get("local_ip"),
            mac=identifier.get("mac"))
        header.scan_settings = mavlink.MAVLink_scan_settings_message(
            yaw_start=scan_settings.get("yaw_start"),
            yaw_stop=scan_settings.get("yaw_stop"),
            pitch_start=scan_settings.get("pitch_start"),
            pitch_stop=scan_settings.get("pitch_stop"),
            pitch_rest_angle=scan_settings.get("pitch_rest_angle"),
            point_spacing=scan_settings.get("point_spacing"),
            scan_speed=scan_settings.get("scan_speed"),
            scan_stop_reasons=scan_settings.get("scan_stop_reasons"))
        header.scan_transform = mavlink.MAVLink_scan_transform_message(
            roll_offset=scan_transform.get("roll_offset"),
            pitch_offset=scan_transform.get("pitch_offset"),
            pitch_scale=scan_transform.get("pitch_scale"),
            yaw_scale=scan_transform.get("yaw_scale"),
            range_scale=scan_transform.get("range_scale"),
            max_range=scan_transform.get("max_range"))
        header.lidar_settings = mavlink.MAVLink_lidar_settings_message(
            update_rate=lidar_settings.get("update_rate"),
            fog_mode_enable=lidar_settings.get("fog_mode_enable"),
            output_disabled_at_boot=lidar_settings.get(
                "output_disabled_at_boot"),
            firmware_version=lidar_settings.get("firmware_version").encode())
        header.pitch_motor_settings = mavlink.MAVLink_motor_settings_message(
            motor=pitch_motor_settings.get("motor"),
            current=pitch_motor_settings.get("current"),
            microsteps=pitch_motor_settings.get("microsteps"),
            gearing_ratio=pitch_motor_settings.get("gearing_ratio"),
            spread_cycle=pitch_motor_settings.get("spread_cycle"),
            pwm_autograd=pitch_motor_settings.get("pwm_autograd"),
            pwm_autoscale=pitch_motor_settings.get("pwm_autoscale"),
            home_offset_steps=pitch_motor_settings.get("home_offset_steps"),
            enforce_minimum_steps=pitch_motor_settings.get(
                "enforce_minimum_steps"),
            steps_to_next_index=pitch_motor_settings.get(
                "steps_to_next_index"),
            usteps_rate=pitch_motor_settings.get("usteps_rate"),
            ustep_angle=pitch_motor_settings.get("ustep_angle"))
        header.yaw_motor_settings = mavlink.MAVLink_motor_settings_message(
            motor=yaw_motor_settings.get("motor"),
            current=yaw_motor_settings.get("current"),
            microsteps=yaw_motor_settings.get("microsteps"),
            gearing_ratio=yaw_motor_settings.get("gearing_ratio"),
            spread_cycle=yaw_motor_settings.get("spread_cycle"),
            pwm_autograd=yaw_motor_settings.get("pwm_autograd"),
            pwm_autoscale=yaw_motor_settings.get("pwm_autoscale"),
            home_offset_steps=yaw_motor_settings.get("home_offset_steps"),
            enforce_minimum_steps=yaw_motor_settings.get(
                "enforce_minimum_steps"),
            steps_to_next_index=yaw_motor_settings.get("steps_to_next_index"),
            usteps_rate=yaw_motor_settings.get("usteps_rate"),
            ustep_angle=yaw_motor_settings.get("ustep_angle"))
        header.scan_result = mavlink.MAVLink_scan_result_info_message(
            type=scan_result.get("type"),
            num_points=scan_result.get("num_points"),
            file_size_bytes=scan_result.get("file_size_bytes"),
            start_time_unix=scan_result.get("start_time_unix"),
            end_time_unix=scan_result.get("end_time_unix"),
            scan_duration=scan_result.get("scan_duration"),
            scan_stop_reason=scan_result.get("scan_stop_reason"),
            scan_start_reason=scan_result.get("scan_start_reason"))
        header.orientation = mavlink.MAVLink_orientation_message(
            roll=orientation.get("roll"),
            pitch=orientation.get("pitch"),
            temp=orientation.get("temp"),
            xmag=orientation.get("xmag"),
            ymag=orientation.get("ymag"),
            zmag=orientation.get("zmag"),
            heading=orientation.get("heading"),
            lat=orientation.get("lat"),
            lon=orientation.get("lon"),
            h_acc=orientation.get("h_acc"),
            v_acc=orientation.get("v_acc"),
            alt=orientation.get("alt"))
        header.average_power = mavlink.MAVLink_power_information_message(
            type=average_power.get("type"),
            current=average_power.get("current"),
            voltage=average_power.get("voltage"),
            power=average_power.get("power"),
            energy_consumed=average_power.get("energy_consumed"),)
        header.minimum_power = mavlink.MAVLink_power_information_message(
            type=minimum_power.get("type"),
            current=minimum_power.get("current"),
            voltage=minimum_power.get("voltage"),
            power=minimum_power.get("power"),
            energy_consumed=minimum_power.get("energy_consumed"),)
        header.maximum_power = mavlink.MAVLink_power_information_message(
            type=maximum_power.get("type"),
            current=maximum_power.get("current"),
            voltage=maximum_power.get("voltage"),
            power=maximum_power.get("power"),
            energy_consumed=maximum_power.get("energy_consumed"),)

        # return the mavlink based header
        return header

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
class RawPolarPoint:
    distance_cm: int
    pitch: int
    yaw: int
    return_strength: int
    raw_bytes: bytes | None
    fmt: str = "<HHHH"

    def to_list(self):
        return [self.distance_cm, self.pitch, self.yaw, self.return_strength]

    def to_bytes(self) -> bytes:
        return struct.pack(self.fmt, self.distance_cm, self.pitch, self.yaw, self.return_strength)

    @classmethod
    def from_bytes(cls, data):
        dist, pitch_raw, yaw_raw, strength = struct.unpack_from(
            RawPolarPoint.fmt, data)
        return RawPolarPoint(distance_cm=dist, pitch=pitch_raw, yaw=yaw_raw, return_strength=strength, raw_bytes=data)

    def to_cartesian(self, roll_offset_deg: float, pitch_offset_deg: float, yaw_scale: float, pitch_scale: float):
        flags = PointFlags.HEALTHY
        if self.distance_cm == 65535:
            # -2, no reply at all from lidar
            flags = PointFlags.NO_RESPONSE
        elif self.distance_cm == 0:
            # no reading from lidar, pointted at sky
            flags = PointFlags.NO_RETURN
        elif self.distance_cm < 100:
            # Hard coded minimum distance below which we shouldn't show
            flags = PointFlags.TOO_CLOSE
        elif self.distance_cm > 15000:
            # Hard coded maximum distance, 150 meters. Above this, dont show.
            flags = PointFlags.TOO_FAR

        if PointFlags.HEALTHY in flags:
            radius_meters = self.distance_cm / 100.0
        else:
            # Doing this will project all the error points onto a 50 cm sphere about the origin, allowing the ability to see where in the orbit points are failing
            radius_meters = 0.5

        pitch_radians = self.pitch / 10000
        yaw_radians = self.yaw / 10000
        roll_offset_radians = math.radians(roll_offset_deg)
        pitch_offset_radians = math.radians(pitch_offset_deg)

        pitch_adjusted_radians = pitch_radians + pitch_offset_radians

        x = math.cos(yaw_radians) * math.sin(pitch_adjusted_radians) * math.cos(
            roll_offset_radians) + math.sin(yaw_radians) * math.sin(roll_offset_radians)
        y = math.sin(yaw_radians) * math.sin(pitch_adjusted_radians) * math.cos(
            roll_offset_radians) - math.cos(yaw_radians) * math.sin(roll_offset_radians)
        z = math.cos(pitch_adjusted_radians) * math.cos(roll_offset_radians)

        x = x * radius_meters
        y = y * radius_meters
        z = z * radius_meters

        return CartesianPoint(x, y, z, self.return_strength, flags)


@dataclass
class CartesianPoint:
    x: float
    y: float
    z: float
    strength: float
    point_flags: PointFlags


@dataclass
class EosV2BinFile:

    file_path: Path | None = None
    data: bytes | None = None

    @classmethod
    def from_file(cls, file_path: Path):
        data: bytes
        with open(file_path, "rb") as file:
            data = file.read()
        print(
            f"Read in {len(data)} bytes from binfile {file_path.as_posix}")
        return EosV2BinFile(file_path=file_path, data=data)

    @classmethod
    def from_bytes(cls, data: bytes):
        return EosV2BinFile(file_path=None, data=data)

    @property
    def preamble(self) -> Preamble:
        if self.data is None:
            return Preamble()
        return Preamble.from_bytes(self.data)

    @property
    def header_bytes(self) -> bytes:
        if self.preamble.header_start is None or self.preamble.header_stop is None or self.data is None:
            return bytes(0)
        return self.data[self.preamble.header_start:self.preamble.header_stop + 1]

    @property
    def notes_bytes(self) -> bytes:
        if self.preamble.notes_start is None or self.preamble.notes_stop is None or self.data is None:
            return bytes(0)
        return self.data[self.preamble.notes_start:self.preamble.notes_stop + 1]

    @property
    def points_bytes(self) -> bytes:
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
        self.points_data = np.array
        self.polar_points: list[RawPolarPoint] = []
        self.cartesian_points: list[CartesianPoint] = []
        self.pcd = None

    @property
    def invalid_points_count(self) -> int:
        res = 0
        for point in self.cartesian_points:
            if PointFlags.NO_RETURN in point.point_flags:
                res += 1
        return res
    
    @property
    def error_points_count(self) -> int:
        res = 0
        for point in self.cartesian_points:
            if PointFlags.NO_RESPONSE in point.point_flags:
                res += 1
        return res

    @property
    def too_close_points_count(self) -> int:
        res = 0
        for point in self.cartesian_points:
            if PointFlags.TOO_CLOSE in point.point_flags:
                res += 1
        return res

    @property
    def points_count(self) -> int:
        return len(self.cartesian_points)

    def _parse_polar_points_from_bytes(self, data: bytes):
        self.polar_points.clear()

        # parse raw bytes into polar points
        for i in range(int(len(data) / 8)):
            point_bytes = data[(8*i):(8*i) + 8]
            self.polar_points.append(RawPolarPoint.from_bytes(point_bytes))

    def create_cartesian_points_from_polar(self):
        self.cartesian_points.clear()
        roll_offet = 0
        pitch_offset = 0
        pitch_scale = 1.0
        yaw_scale = 1.0

        if self.header is not None:
            if self.header.scan_transform is not None:
                transform = self.header.scan_transform
                roll_offet = transform.roll_offset
                pitch_offset = transform.pitch_offset
                pitch_scale = transform.pitch_scale
                yaw_scale = transform.yaw_scale

        for polar_point in self.polar_points:
            self.cartesian_points.append(polar_point.to_cartesian(
                roll_offset_deg=roll_offet, pitch_offset_deg=pitch_offset, yaw_scale=yaw_scale, pitch_scale=pitch_scale))


    # Parse polar points into cartesian points and save to a pcd

    def make_pcd(self, filename: str, with_polar: bool = True, with_invalid_points=True):
        # All versions have xyzi fields
        fields = ("x", "y", "z", "intensity")
        types = (np.float32, np.float32, np.float32, np.int16)
        arr = []

        # Add polar values if requested
        if with_polar:
            fields = fields + ("pitch", "yaw", "distance",)
            types = types + (np.uint16, np.uint16, np.uint16,)

        # Add flags if requested
        if with_invalid_points:
            fields = fields + ("flags",)
            types = types + (np.uint8,)

        for index, c_point in enumerate(self.cartesian_points):
            point = None
            point = [c_point.x, c_point.y, c_point.z,
                     c_point.strength]  # default values
            if PointFlags.HEALTHY not in c_point.point_flags and not with_invalid_points:
                # don't add the point if it's not healthy and we've been asked to ignore it.
                continue

            # Add polar values if requested
            if with_polar:
                p_point = self.polar_points[index]
                point.extend([p_point.pitch, p_point.yaw, p_point.distance_cm])

            # Add flags if requested
            if with_invalid_points:
                point.extend([c_point.point_flags.value])

            if point is not None:
                arr.append(point)

        # Write the header as a comment to the top line of the PCD
        v_file = io.BytesIO()
        header = {"header": self.header.to_dict_annotated(),
                  "notes": self.notes}
        v_file.write(f"#{simplejson.dumps(header, ignore_nan=True)}".encode())
        v_file.write(f"\n\r".encode())

        np_arr = np.array(arr)
        pcd = PointCloud.from_points(np_arr, fields, types)
        pcd.save(v_file, encoding=Encoding.BINARY_COMPRESSED)
        with open(filename, mode="wb") as pcd_file:
            pcd_file.write(v_file.getvalue())

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
    def _generate_preamble(cls, header: Header, notes: str, points: bytearray) -> Preamble:
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

    @classmethod
    def _create_points_bytes_from_pcd(cls, point_cloud: PointCloud) -> bytearray:
        points_bytes = bytearray()

        for point in point_cloud.pc_data:
            ppoint = RawPolarPoint(
                distance_cm=point[6], pitch=point[4], yaw=point[5], return_strength=point[3], raw_bytes=None)
            points_bytes += ppoint.to_bytes()
        return points_bytes

    @classmethod
    def from_pcd(cls, path_to_file: Path):
        bin_file = EOSV2Scan._pcd_to_binfile(path_to_file)
        return EOSV2Scan.from_bin_file(bin_file)

    @classmethod
    def from_bin_file(cls, bin_file: EosV2BinFile):
        scan = EOSV2Scan()
        scan.bin_file = bin_file
        scan._parse_binfile_data()
        return scan

    @classmethod
    def from_path(cls, path_to_file: Path):
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
            self.create_cartesian_points_from_polar()

    def to_json(self) -> str:
        d = self.header.to_dict_annotated()
        polar_points_list = []
        for point in self.polar_points:
            polar_points_list.append(point.to_list())
        d["NOTES"] = self.notes
        d["POINTS"] = polar_points_list
        j = simplejson.dumps(d, ignore_nan=True)
        return j


def pack_mavlink_msg_payload(msg: mavlink.MAVLink_message) -> bytes:
    args = []
    for field in msg.ordered_fieldnames:
        foo = getattr(msg, field)
        if type(foo) is str:
            args.append(foo.encode())
        elif type(foo) is list:
            for entry in foo:
                args.append(entry)
        else:
            args.append(foo)

    foo = msg.unpacker.pack(*args)
    return foo
