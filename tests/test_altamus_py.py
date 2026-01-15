import unittest
from altamus_py.scan import EOSV2Scan, Header, PointFlags
from pathlib import Path
import tempfile
import simplejson
import os

class TestFileGeneration(unittest.TestCase):
    def setUp(self) -> None:
        self.file = Path("./tests/sample_files/batch_plant.bin")
        self.temp_dir = tempfile.TemporaryDirectory()
        self.temp_dir_path = Path(self.temp_dir.name)
        print("setting up for tests")

    def tearDown(self) -> None:
        print("Tearing down")

    def test_load_scan_from_bin(self):
        scan = EOSV2Scan.from_path(self.file.absolute())
        self._test_sample_scan_header(scan.header)
        self._test_sample_scan_properties(scan)
        
    def test_create_full_pcd_from_bin(self):
        pcd_path = self.temp_dir_path / "all_points.pcd"
        scan = EOSV2Scan.from_path(self.file.absolute())
        scan.save_annotated_pcd_to_file(pcd_path)
        
        self.assertTrue(os.path.exists(pcd_path))
        with open(pcd_path, "rb") as f:
            data = f.read()
        self.assertEqual(len(data), 2317202)
        self.assertEqual(data[0], 35) # first character is a hash #
        self.assertEqual(data[10000], 135) # randomly picked byte
        self.assertEqual(data[20000], 27) # randomly picked byte
    
    def test_create_filtered_pcd_from_bin(self):
        pcd_path = self.temp_dir_path / "healthy_only.pcd"
        scan = EOSV2Scan.from_path(self.file.absolute())
        scan.save_annotated_pcd_to_file(pcd_path, include_error_points=False)
        
        self.assertTrue(os.path.exists(pcd_path))
        with open(pcd_path, "rb") as f:
            data = f.read()
        self.assertEqual(len(data), 1989540)
        self.assertEqual(data[0], 35) # first character is a hash #
        self.assertEqual(data[10000], 70) # randomly picked byte
        self.assertEqual(data[20000], 30) # randomly picked byte

    def test_create_scan_from_pcd(self):
        # create the pcd
        pcd_path = self.temp_dir_path / "test.pcd"
        scan = EOSV2Scan.from_path(self.file.absolute())
        scan.save_annotated_pcd_to_file(pcd_path)

        # read in the pcd
        scan = EOSV2Scan.from_pcd(pcd_path)
        self._test_sample_scan_header(scan.header)
        self._test_sample_scan_properties(scan)

    def test_create_polar_json_for_hyperion(self):
        scan = EOSV2Scan.from_path(self.file.absolute())
        json_str = scan.polar_points_to_json()
        json = simplejson.loads(json_str)
        points = json["POINTS"]
        self.assertEqual(scan.points_count, len(points))

    def test_filtered_points(self):
        scan = EOSV2Scan.from_path(self.file.absolute())
        healthy_only = scan.get_filtered_points([PointFlags.HEALTHY])
        self.assertTrue(len(healthy_only), scan.healthy_points_count)

        healthy_and_too_far = scan.get_filtered_points([PointFlags.HEALTHY, PointFlags.TOO_FAR])
        self.assertTrue(len(healthy_and_too_far), scan.healthy_points_count + scan.too_far_points_count)
    
    def _test_sample_scan_header(self, header: Header):
        # this test is specific to the provided sample "batch_plant.bin" file. If the file changes these might fail
        self.assertIsNotNone(header)
        self.assertIsNotNone(header.identifier)
        self.assertIsNotNone(header.lidar_settings)
        self.assertIsNotNone(header.scan_settings)
        self.assertIsNotNone(header.orientation)
        if(header.orientation is None or header.identifier is None or header.scan_settings is None or header.lidar_settings is None):
            return

        # Pick a few random attributes as a spot check that the header is parsed correctly
        self.assertEqual(header.identifier.device_id, "P2-029C10")
        self.assertEqual(header.identifier.mac, [148, 148, 74, 2, 156, 16])
        self.assertEqual(header.lidar_settings.update_rate, 1000)
        self.assertEqual(header.scan_settings.pitch_start, 80)
        self.assertEqual(header.scan_settings.pitch_stop, 280)
        self.assertEqual(header.orientation.lat, 257940928)
    
    def _test_sample_scan_properties(self, scan: EOSV2Scan):
        # check the points health categories
        self.assertEqual(scan.points_count, 163608)
        self.assertEqual(scan.healthy_points_count, 133665)
        self.assertEqual(scan.no_response_points_count, 8)
        self.assertEqual(scan.no_return_points_count, 20105)
        self.assertEqual(scan.too_close_points_count, 3893)
        self.assertEqual(scan.too_far_points_count, 5937)

        # totaling up all the categories of points should equal the raw total amount of points
        self.assertEqual(scan.healthy_points_count + scan.no_response_points_count + scan.no_return_points_count + scan.too_close_points_count + scan.too_far_points_count, scan.points_count)

        # check some values from the overlap extraction property
        overlap, start, stop = scan.yaw_overlap_points
        self.assertAlmostEqual(overlap, 20.1910327)
        self.assertEqual(start.size, 133496)
        self.assertEqual(stop.size, 132696)

        # total number of cartesian points should match the scan points
        cartesian = scan.cartesian_points
        self.assertEqual(len(cartesian), scan.points_count)

if __name__ == "__main_":
    unittest.main()