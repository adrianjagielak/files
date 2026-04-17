"""
Pre-build script: inject the nanopb include directory into CPPPATH so that
tesla-ble sources added via build_src_filter can resolve <pb.h>.

PlatformIO's lib_deps include paths are not always propagated to source files
pulled in through build_src_filter from outside src/, so we add the path
explicitly here after lib_deps have been installed.
"""
Import("env")
import os, glob

libdeps = os.path.join(env["PROJECT_DIR"], ".pio", "libdeps", env["PIOENV"])

for candidate in glob.glob(os.path.join(libdeps, "[Nn]anopb*")):
    if os.path.isfile(os.path.join(candidate, "pb.h")):
        env.Append(CPPPATH=[candidate])
        print("extra_scripts: nanopb include path added:", candidate)
        break
