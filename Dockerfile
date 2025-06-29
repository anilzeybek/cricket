FROM mambaorg/micromamba:2.3.0
ENV DEBIAN_FRONTEND=noninteractive

RUN micromamba create -n cricket -c conda-forge pinocchio cppad eigen cgal inja nlohmann_json cmake cxx-compiler llvm ninja fmt pkg-config
SHELL ["micromamba", "run", "-n", "cricket", "/bin/bash", "-c"]

COPY ./CppADCodeGen CppADCodeGen
RUN cmake -GNinja -DCMAKE_INSTALL_PREFIX=cppadcg_install -Bcppadcg_build CppADCodeGen/ && cmake --build cppadcg_build && cmake --install cppadcg_build

RUN mkdir -p cricket
COPY ./src cricket/src
COPY ./CMakeLists.txt cricket/CMakeLists.txt
RUN cmake -GNinja -Bcricket_build  -DCMAKE_PREFIX_PATH="cppadcg_install" cricket/ && cmake --build cricket_build
