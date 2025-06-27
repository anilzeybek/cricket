cmake -GNinja -Bcppadcg_build -DCMAKE_PREFIX_PATH=/home/zak/micromamba/envs/cricket/ -DCMAKE_INSTALL_PREFIX=./cppadcg_install CppADCodeGen/
cmake --install cppadcg_build

cmake -GNinja -Bpinocchio_build -DCMAKE_PREFIX_PATH=/home/zak/micromamba/envs/cricket/ -DCMAKE_INSTALL_PREFIX=./pinocchio_install -DBUILD_WITH_CODEGEN_SUPPORT=On -Dcppadcg_PREFIX=${PWD}/cppadcg_install/include pinocchio/
