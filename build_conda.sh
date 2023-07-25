#!/bin/bash -xe

# mamba install anaconda-client
ANACONDA_TOKEN=XXXXX
anaconda -t $ANACONDA_TOKEN upload --user fair-robotics $CONDA_PREFIX/conda-bld/linux-64/*.tar.bz2 --force
