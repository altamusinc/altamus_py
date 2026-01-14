from altamus_py.scan import EOSV2Scan
from pathlib import Path

# parse a pcd into a scan file
file = Path("./tests/sample_files/batch_plant.pcd")
scan = EOSV2Scan.from_pcd(file)

# Re-export that same scan file to another PCD. should match the original pcd
pcd_filename = f"{file.stem}_from_pcd.pcd"
scan.save_pcd_to_file(Path.joinpath(file.parent, pcd_filename).as_posix())
