#!/bin/bash
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=24
#SBATCH --cluster=smp
#SBATCH --partition=smp
#SBATCH --time=1:00:00
#SBATCH --job-name=hello

# Load Modules
module purge
module load intel/2017.3.196

# OMP_NUM_THREADS is not set
echo
echo " ====== OMP_NUM_THREADS = " $OMP_NUM_THREADS
./hello_world_id.x


# OMP_NUM_THREADS = 2
export OMP_NUM_THREADS=2
echo
echo " ====== OMP_NUM_THREADS = " $OMP_NUM_THREADS
./hello_world_id.x


# OMP_NUM_THREADS = 12
export OMP_NUM_THREADS=12
echo
echo " ====== OMP_NUM_THREADS = " $OMP_NUM_THREADS
./hello_world_id.x


# OMP_NUM_THREADS = 24
export OMP_NUM_THREADS=24
echo
echo " ====== OMP_NUM_THREADS = " $OMP_NUM_THREADS
./hello_world_id.x


# OMP_NUM_THREADS = 48.  Note that your can over-subscribe the physical cores.
export OMP_NUM_THREADS=48
echo
echo " ====== OMP_NUM_THREADS = " $OMP_NUM_THREADS
./hello_world_id.x



