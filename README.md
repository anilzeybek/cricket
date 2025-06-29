Compilation notes, needs cleaned.

Create environment with pinocchio, cppad, eigen, and cgal.
Remove pinocchio since we are compiling it ourselves.
```bash
micromamba create -n cricket -c conda-forge python=3.12 pinocchio cppad eigen cgal
micromamba remove -f pinocchio
```

Activate env.
```bash
micromamba activate cricket
```

Build CppADCodeGen.
```bash
cmake -GNinja -Bcppadcg_build -DCMAKE_PREFIX_PATH=/home/zak/micromamba/envs/cricket/ -DCMAKE_INSTALL_PREFIX=${PWD}/cppadcg_install CppADCodeGen/
cmake --install cppadcg_build
```

Build Pinocchio with codegen capabilities.
Watch out for the job count - start low.
Will OoM your machine.
```bash
cmake -GNinja -Bpinocchio_build -DCMAKE_PREFIX_PATH=/home/zak/micromamba/envs/cricket/ -DCMAKE_INSTALL_PREFIX=${PWD}/pinocchio_install -DBUILD_WITH_CODEGEN_SUPPORT=On -Dcppadcg_PREFIX=${PWD}/cppadcg_install/include -DBUILD_UNIT_TESTS=Off -DBUILD_WITH_COLLISION_SUPPORT=On pinocchio/
cmake --build pinocchio_build -j20
cmake --install pinocchio_build
```

Build the script.
```bash
cmake -GNinja -Bbuild -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="/home/zak/micromamba/envs/cricket/;${PWD}/pinocchio_install;${PWD}/cppadcg_install" .
cmake --build build
```

Run the script.
Optionally format the code.
```bash
./build/fkcc_gen panda.json
clang-format -i panda_fk.hh
```
