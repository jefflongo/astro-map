import astropy.units as u
import numpy as np
from astropy.coordinates import Angle
from astroquery.vizier import Vizier

print("Loading Bright Star Catalog... ", end="", flush=True)
v = Vizier(columns=["RAJ2000", "DEJ2000", "Vmag"], row_limit=-1)
catalog = v.get_catalogs("V/50")[0]
valid = (
    (catalog["RAJ2000"] != "") & (catalog["DEJ2000"] != "") & (~catalog["Vmag"].mask)
)
catalog = catalog[valid]
print(f"{len(catalog)} stars found")

ra = Angle(catalog["RAJ2000"], unit=u.hourangle).radian
dec = Angle(catalog["DEJ2000"], unit=u.deg).radian
vmag = np.array(catalog["Vmag"], dtype=np.float64)

with open("bsc.h", "w", encoding="utf-8") as f:
    f.write("#pragma once\n\n")
    f.write(f"#define BSC_SIZE {len(catalog)}\n\n")
    f.write("extern double const _BSC_RA[];\n")
    f.write("extern double const _BSC_DEC[];\n")
    f.write("extern double const _BSC_VMAG[];\n")

with open("bsc.c", "w", encoding="utf-8") as f:

    def format_array(arr):
        lines = []
        for i in range(0, len(arr), 4):
            chunk = arr[i : i + 4]
            line = ", ".join(f"{x:.17g}" for x in chunk)
            lines.append("    " + line)
        return ",\n".join(lines)

    f.write("double const _BSC_RA[] = {\n")
    f.write(format_array(ra))
    f.write("\n};\n\n")

    f.write("double const _BSC_DEC[] = {\n")
    f.write(format_array(dec))
    f.write("\n};\n\n")

    f.write("double const _BSC_VMAG[] = {\n")
    f.write(format_array(vmag))
    f.write("\n};\n")
