#!/bin/bash

echo "Building cython_min_cost_path.pyx..."
cython3 ./cython_min_cost_path.pyx

PY_INCLUDE_PATH=$(python3 -c 'from sysconfig import get_paths as gp; print(gp()["include"])')
echo "Using python lib: $PY_INCLUDE_PATH"
gcc -shared -pthread -fPIC -fwrapv -ffast-math -O3 -Wall -fno-strict-aliasing -I$PY_INCLUDE_PATH -o cython_min_cost_path.so cython_min_cost_path.c
echo "Done."
echo ""

# In the future, consider the following recommendation from the Cython
# documentation found at
# https://cython.readthedocs.io/en/latest/src/quickstart/build.html
#
# "Write a setuptools setup.py. This is the normal and recommended way."
#

# In the future try adding -DNPY_NO_DEPRECATED_API=NPY_1_7_API_VERSION to address the following error (adding this argument currently results in compilation failure). 
#
# /usr/include/python2.7/numpy/npy_1_7_deprecated_api.h:15:2: warning: #warning "Using deprecated NumPy API, disable it by " "#defining NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION" [-Wcpp]
