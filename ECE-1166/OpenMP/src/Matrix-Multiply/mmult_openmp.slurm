#!/bin/bash
#SBATCH --job-name=mattis_mmult_openmp
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=4
#SBATCH --cluster=smp
#SBATCH --partition=smp
#SBATCH --mail-user=zachary.mattis@pitt.edu
#SBATCH --mail-type=FAIL
#SBATCH --time=1:00:00
#SBATCH --qos=short

# Load Modules
module purge
module load intel/2017.3.196

# Env Variables
echo "SLURM_JOB_NAME = " $SLURM_JOB_NAME
echo "SLURM_NTASKS = " $SLURM_NTASKS
echo "SLURM_JOB_ID = " $SLURM_JOB_ID

# OMP_NUM_THREADS = 4
export OMP_NUM_THREADS=4
echo "OMP_NUM_THREADS = " $OMP_NUM_THREADS
./mmult_openmp.x