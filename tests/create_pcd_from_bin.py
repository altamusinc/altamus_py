import argparse
from pathlib import Path
from altamus_py.scan import EOSV2Scan

parser = argparse.ArgumentParser()
parser.add_argument('-f', "--file", type=Path)
parser.add_argument('-o', '--output', type=Path)
parser.add_argument('-p', '--create-pcd', type=bool)
args = parser.parse_args()
file = args.file

if file is None:
    file = Path("./tests/sample_files/batch_plant.bin")

scan = EOSV2Scan.from_binfile(file.absolute())

scan.make_pcd("foobar.pcd")
print(scan.to_json())
