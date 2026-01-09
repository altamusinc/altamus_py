from altamus_py.scan import EOSV2Scan
from pathlib import Path

file = Path("./tests/sample_files/batch_plant.pcd")
scan = EOSV2Scan.from_pcd(file)
print(scan)