#!/bin/bash
./build/fkcc_gen resources/ur5.json
clang-format -i ur5_fk.hh
cp ur5_fk.hh ../vamp/src/impl/vamp/robots/ur5.hh
cp resources/ur5/ur5_spherized.urdf ../vamp/resources/ur5/ur5_spherized.urdf

./build/fkcc_gen resources/panda.json
clang-format -i panda_fk.hh
cp panda_fk.hh ../vamp/src/impl/vamp/robots/panda.hh

./build/fkcc_gen resources/fetch.json
clang-format -i fetch_fk.hh
cp fetch_fk.hh ../vamp/src/impl/vamp/robots/fetch.hh

./build/fkcc_gen resources/baxter.json
clang-format -i baxter_fk.hh
cp baxter_fk.hh ../vamp/src/impl/vamp/robots/baxter.hh
