







micromamba create -n cricket -c conda-forge python=3.12 pinocchio cppad eigen
micromamba remove -f pinocchio

cmake -GNinja -Bcppadcg_build -DCMAKE_PREFIX_PATH=/home/zak/micromamba/envs/cricket/ -DCMAKE_INSTALL_PREFIX=${PWD}/cppadcg_install CppADCodeGen/
cmake --install cppadcg_build

cmake -GNinja -Bpinocchio_build -DCMAKE_PREFIX_PATH=/home/zak/micromamba/envs/cricket/ -DCMAKE_INSTALL_PREFIX=${PWD}/pinocchio_install -DBUILD_WITH_CODEGEN_SUPPORT=On -Dcppadcg_PREFIX=${PWD}/cppadcg_install/include -DBUILD_UNIT_TESTS=Off -DBUILD_WITH_COLLISION_SUPPORT=On pinocchio/
cmake --build pinocchio_build -j20
cmake --install pinocchio_build


export PYTHONPATH="${PWD}/pinocchio_install/lib:${PWD}/example_data_install/lib:$PYTHONPATH"


cmake -GNinja -Bexample_data_build -DCMAKE_PREFIX_PATH="/home/zak/micromamba/envs/cricket/;${PWD}/install" -DCMAKE_INSTALL_PREFIX=${PWD}/install pinocchio/models/example-robot-data/
cmake --build example_data_build
cmake --install example_data_build

cmake -GNinja -Bbuild -DCMAKE_PREFIX_PATH="/home/zak/micromamba/envs/cricket/;${PWD}/pinocchio_install;${PWD}/cppadcg_install" .
