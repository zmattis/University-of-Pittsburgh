#!/bin/bash
#SBATCH --nodes=2
#SBATCH --ntasks-per-node=28
#SBATCH --cluster=mpi
#SBATCH --partition=opa
#SBATCH --time=1:00:00
#SBATCH --job-name=hello

# Load Modules
module purge
module load intel/2017.3.196

echo "SLURM_NTASKS = " $SLURM_NTASKS

# Execute using standard mpirun
mpirun -n $SLURM_NTASKS hello_world.x >& hello_world_intel.out

# Alternative: Execute using SLURM's srun
# srun --mpi=pmi2 hello_world.x >& hello_world_intel.out
