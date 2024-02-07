#!/bin/bash
# SPDX-FileCopyrightText: 2023-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-FileCopyrightText: 2023-2024 Sotiris Papatheodorou
# SPDX-License-Identifier: CC0-1.0

# Slurm batch script for testing supereight2. To start a Slurm job run:
#   sbatch supereight-slurm-test.bash

#SBATCH --job-name="se2_tests"
#SBATCH --cpus-per-task=8
#SBATCH --gpus-per-task=0
#SBATCH --mem=16G
#SBATCH --time=1:00:00
#SBATCH --mail-type=END,FAIL
#SBATCH --output=/storage/slurm/%u/%x/%j/slurm.log
#SBATCH --error=/storage/slurm/%u/%x/%j/slurm.log

# The directory where the supereight2 repository is located. Make sure the
# commit you want to test is already checked out and all submodules have the
# appropriate commits checked out.
supereight2_dir="/usr/wiss/$USER/test/supereight-2-srl"

# The directory where output files will be written.
output_dir="/storage/slurm/$USER/$SLURM_JOB_NAME/$SLURM_JOB_ID"

srun "$supereight2_dir/scripts/supereight-test.bash" "$output_dir"
