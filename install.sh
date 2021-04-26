rm -rf build dist
rm fot_planner.cpython-38-x86_64-linux-gnu.so
./build.sh
python setup.py build_ext --inplace