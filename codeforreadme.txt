

sudo rosdep init
rosdep update --include-eol-distros
rosdep install --from-paths src -y --ignore-src




git submodule update --recursive --init

cd ~/drone-software/src/trajectory_gen/acados
mkdir -p build
cd build
cmake -DACADOS_WITH_QPOASES=ON ..
make install -j4
cd ~/drone-software/



sudo apt install python3-pip
cd ~/drone-software/src/trajectory_gen/acados/
pip install -e interfaces/acados_template
cd ~/drone-software/

source setup_acados.sh
cd ~/drone-software
python3 src/trajectory_gen/acados/examples/acados_python/getting_started/minimal_example_ocp.py 
