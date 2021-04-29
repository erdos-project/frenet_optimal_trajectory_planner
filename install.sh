rm -rf build dist
rm fot_planner.cpython-38-x86_64-linux-gnu.so
rm /home/simonguozirui/anaconda3/envs/erdos/lib/python3.8/site-packages/fot_planner-1.0.0-py3.8-linux-x86_64.egg/fot_planner.cpython-38-x86_64-linux-gnu.so
./build.sh
# python setup.py build_ext --inplace
python setup.py install