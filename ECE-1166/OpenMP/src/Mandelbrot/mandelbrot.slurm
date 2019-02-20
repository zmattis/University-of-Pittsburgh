#!/bin/bash
#SBATCH --job-name=mattis_mandelbrot
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=8
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

# OMP_NUM_THREADS = 8
export OMP_NUM_THREADS=8
echo "OMP_NUM_THREADS = " $OMP_NUM_THREADS
./mandelbrot.x mandelbrot.out 2048 0