# 0.1.0

- Initial release

# 0.1.1

- Add property for expected number of points

# 0.1.2

- Add unhealthy points count property to scan

# 0.1.3

- Individual header values can no longer be None, throw value error if parsing any doesn't work
- Add missing points count property
- Fix yaw overlap points exploding when the scan doesn't have any overlap
- Fix expected points count to return the actual points count when scan was canceled
- add `calculated_expected_points_count` to explicitly return the points based off the settings value

# 0.1.4

- Add a lot of getters for common values stored in the header
- Add handling for additional cartesian transform after the initial calibration transform
- cache the cartesian coordinates instead of making them a property

# 0.1.5

- Add ability to switch cartesian units between feet and meters
- Make "notes" property cat the power stats on for backwards compatibility with hyperion