import argparse
import pathlib
import re
import shutil

LIBMATH_FNS = [
    "acos",
    "acosh",
    "asin",
    "asinh",
    "atan",
    "atan2",
    "atanh",
    "cbrt",
    "ceil",
    "copysign",
    "cos",
    "cosh",
    "erf",
    "erfc",
    "exp",
    "exp2",
    "expm1",
    "fabs",
    "fdim",
    "floor",
    "fmax",
    "fmin",
    "fmod",
    "frexp",
    "hypot",
    "ilogb",
    "ldexp",
    "lgamma",
    "log",
    "log10",
    "log1p",
    "log2",
    "logb",
    "modf",
    "nan",
    "nearbyint",
    "nextafter",
    "nexttoward",
    "pow",
    "remainder",
    "remquo",
    "rint",
    "round",
    "scal",
    "scalbln",
    "scalbn",
    "sin",
    "sinh",
    "sqrt",
    "tan",
    "tanh",
    "tgamma",
    "trunc",
]

LIBMATH_CONSTANTS = [
    "_DIG",
    "_EPSILON",
    "_MANT_DIG",
    "_MAX_10_EXP",
    "_MAX_EXP",
    "_MAX",
    "_MIN_10_EXP",
    "_MIN_EXP",
    "_MIN",
]

LIBMATH_FNS_RE = [(re.compile(rf"\b{fn}\b"), f"{fn}f") for fn in LIBMATH_FNS]
LIBMATH_CONSTANTS_RE = [
    (re.compile(rf"\bDBL{constant}\b"), f"FLT{constant}")
    for constant in LIBMATH_CONSTANTS
]
DOUBLE_TYPE_RE = re.compile(r"\bdouble\b")
DOUBLE_LITERAL_RE = re.compile(
    r"""
    (?<![\w.])               # not preceded by letter, digit, underscore, or dot
    (                        # capture this whole literal:
      (?:                    # either..
         \d*\.\d+            #  at least one digit after the decimal point
       | \d+\.\d*            #  at least one digit before the decimal point
       | \d+[eE][+-]?\d+     #  integer with scientific notation exponent
      )
      (?:[eE][+-]?\d+)?      # optional scientific notation exponent for the first two cases
    )
    (?![fF\w.])              # not followed by f/F, letter, digit, underscore, or dot
    """,
    re.VERBOSE,
)

parser = argparse.ArgumentParser()
parser.add_argument("path", help="Path to C library to convert to single precision")
args = parser.parse_args()

library_path = pathlib.Path(args.path)
if not library_path.exists():
    raise SystemExit("ERROR: library path does not exist, exiting")

shutil.rmtree(library_path.stem, ignore_errors=True)

for root, _, files in library_path.walk():
    root.relative_to(library_path.parent).mkdir()

    for name in files:
        src_path = pathlib.Path(root, name)
        dest_path = src_path.relative_to(library_path.parent)

        if src_path.suffix in (".c", ".h"):
            with open(src_path, "r", encoding="utf-8") as src_file:
                content = src_file.read()

            # replace math.h functions with single-precision variants
            for pattern, replace in LIBMATH_FNS_RE:
                content = pattern.sub(replace, content)

            # replace math.h constants with single-precision variants
            for pattern, replace in LIBMATH_CONSTANTS_RE:
                content = pattern.sub(replace, content)

            # replace the "double" keyword with "float"
            content = DOUBLE_TYPE_RE.sub("float", content)

            # postfix double literals with "f"
            content = DOUBLE_LITERAL_RE.sub(lambda m: f"{m.group(1)}f", content)

            with open(dest_path, "w", encoding="utf-8") as dest_file:
                dest_file.write(content)
        else:
            shutil.copy(src_path, dest_path)
