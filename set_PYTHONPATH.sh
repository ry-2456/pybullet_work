#!/bin/bash

# set PYTHONPATH for pybullet
# Note that bullet3 and pybullet_work must be in the same directory.

# reference
# bash/zshでsourceされたスクリプト内で~ https://qiita.com/yudoufu/items/48cb6fb71e5b498b2532 

bullet_module_path="/bullet3/build_cmake/examples/pybullet"            # path to pybullet modules
source_dir=$(cd $(dirname -- ${BASH_SOURCE:-0}); pwd)                  # set_PYTHONPATHのpath
export PYTHONPATH=$(cd ${source_dir} && cd ../ && pwd)${bullet_module_path}

# print PYTHONPATH
yes = | head -n ${#PYTHONPATH} | tr -d '\n'
echo
echo "set PYTHONPATH as follows"
echo ${PYTHONPATH}
yes = | head -n ${#PYTHONPATH} | tr -d '\n'
echo
