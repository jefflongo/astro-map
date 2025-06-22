import argparse

import astropy.units as u
import numpy as np
from astropy.coordinates import Angle
from astroquery.vizier import Vizier

parser = argparse.ArgumentParser()
parser.add_argument("vmag", type=float, help="Maximum visual magnitude to output")
parser.add_argument(
    "--single-precision",
    action="store_true",
    dest="sp",
    help="Output single precision floats",
)
args = parser.parse_args()

TYPE_STRING = "float" if args.sp else "double"

v = Vizier(catalog="V/50", columns=["RAJ2000", "DEJ2000", "Vmag"], row_limit=-1)
catalog = v.query_constraints(Vmag=f"<={args.vmag}")[0]
valid = (
    (catalog["RAJ2000"] != "") & (catalog["DEJ2000"] != "") & (~catalog["Vmag"].mask)
)
catalog = catalog[valid]
catalog.sort("Vmag")

ra = Angle(catalog["RAJ2000"], unit=u.hourangle).radian
dec = Angle(catalog["DEJ2000"], unit=u.deg).radian
vmag = np.array(catalog["Vmag"], dtype=np.float32 if args.sp else np.float64)

with open("bsc.h", "w", encoding="utf-8") as f:
    f.write("#pragma once\n\n")
    f.write(f"#define BSC_SIZE {len(catalog)}\n\n")
    f.write(f"extern {TYPE_STRING} const _BSC_RA[];\n")
    f.write(f"extern {TYPE_STRING} const _BSC_DEC[];\n")
    f.write(f"extern {TYPE_STRING} const _BSC_VMAG[];\n")

with open("bsc.c", "w", encoding="utf-8") as f:

    def format_array(arr):
        lines = []
        for i in range(0, len(arr), 4):
            chunk = arr[i : i + 4]
            line = ", ".join(f"{x:.17f}{'f' if args.sp else ''}" for x in chunk)
            lines.append("    " + line)
        return ",\n".join(lines)

    f.write(f"{TYPE_STRING} const _BSC_RA[] = {{\n")
    f.write(format_array(ra))
    f.write("\n};\n\n")

    f.write(f"{TYPE_STRING} const _BSC_DEC[] = {{\n")
    f.write(format_array(dec))
    f.write("\n};\n\n")

    f.write(f"{TYPE_STRING} const _BSC_VMAG[] = {{\n")
    f.write(format_array(vmag))
    f.write("\n};\n")
