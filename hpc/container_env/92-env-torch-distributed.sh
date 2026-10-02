#!/bin/sh

# Copyright (c) 2018-2019, NVIDIA CORPORATION. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Set environment variables used by PyTorch Distributed


###############
## Get Master address and port from SLURM.
##
## Note: It has to match the address of the rank that will become RANK=0 for PyTorch Distributed to work.
## TODO: allow getting these directly from OpenMPI in case we're not running under SLURM
##
if [ -z "${MASTER_ADDR:-}" -a -n "${SLURM_NODELIST:-}" ]; then
    # 'hostlist' is from the python-hostlist package.  Could use scontrol
    # here, but it's not available inside the container.
    export MASTER_ADDR="$(hostlist -l 1 "${SLURM_NODELIST}")"
fi
if [ -z "${MASTER_PORT:-}" ]; then
    # Avoid a Bash subprocess: BASH_ENV triggers NVIDIA driver probing.
    export MASTER_PORT="$((${SLURM_JOB_ID:-4242} % 16384 + 49152))"
fi

## Get world size from OpenMPI or alternatively from SLURM
if [ -z "${WORLD_SIZE:-}" ]; then
    if [ -n "${OMPI_COMM_WORLD_SIZE:-}" ]; then
        export WORLD_SIZE="${OMPI_COMM_WORLD_SIZE}"
    elif [ -n "${SLURM_NTASKS:-}" ]; then
        export WORLD_SIZE="${SLURM_NTASKS}"
    fi
fi

## Get (global) process ID from OpenMPI or alternatively from SLURM
if [ -z "${RANK:-}" ]; then
    if [ -n "${OMPI_COMM_WORLD_RANK:-}" ]; then
        export RANK="${OMPI_COMM_WORLD_RANK}"
    elif [ -n "${SLURM_PROCID:-}" ]; then
        export RANK="${SLURM_PROCID}"
    fi
fi

## Get local process ID from OpenMPI or alternatively from SLURM
if [ -z "${LOCAL_RANK:-}" ]; then
    if [ -n "${OMPI_COMM_WORLD_LOCAL_RANK:-}" ]; then
        export LOCAL_RANK="${OMPI_COMM_WORLD_LOCAL_RANK}"
    elif [ -n "${SLURM_LOCALID:-}" ]; then
        export LOCAL_RANK="${SLURM_LOCALID}"
    fi
fi
