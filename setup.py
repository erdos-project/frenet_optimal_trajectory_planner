from setuptools import Distribution
import setuptools.command.build_ext as _build_ext
from distutils.core import setup, Extension
import sysconfig
from pathlib import Path
import sys, os
import subprocess

class build_ext(_build_ext.build_ext):
    def run(self):
            command = ["./build.sh", "-p", sys.executable]
            subprocess.check_call(command)

class BinaryDistribution(Distribution):
    def has_ext_modules(self):
        return True

def main():
    os.environ['CC'] = 'g++'
    os.environ['CXX'] = 'g++'
    
    # You may find this reference helpful: https://docs.python.org/3.6/extending/building.html
    pwd = os.path.dirname(os.path.realpath(__file__))    
    CFLAGS = ['-std=c++11', '-fPIC', '-shared', '-I', '-Ofast', '-pthread', '-Wall', '-I/usr/include/eigen3']
    LDFLAGS = []
    LDFLAGS += ['-Xlinker', '-export-dynamic']
    LDFLAGS += ['-I', '/usr/include/x86_64-linux-gnu/qt5/QtCore', '-l' , 'Qt5Core']
    LDFLAGS += ['-L' + os.path.join(pwd, 'build'), '-lPackageFrenetOptimalTrajectory']
    print("linker flag", LDFLAGS)

    module = Extension('fot_planner',
                    sources = ['src/FrenetOptimalTrajectory/planner_package.cpp'],
                    language='C++',
                    include_dirs=['src', 'src/CubicSpline', 'src/Polynomials', 'src/FrenetOptimalTrajectory', 'src/Obstacle', 'src/Car'],
                    extra_compile_args = CFLAGS, 
                    extra_link_args= LDFLAGS)

    setup(name="fot_planner",
        version="1.0.0",
        description="FOT Planner",
        author="ERDOS Project",
        ext_modules=[module])

if __name__ == "__main__":
    main()