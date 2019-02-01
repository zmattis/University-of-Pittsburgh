#!/bin/bash
#SBATCH --nodes=2
#SBATCH --ntasks-per-node=28
#SBATCH --cluster=mpi
#SBATCH --partition=opa
#SBATCH --time=1:00:00
#SBATCH --job-name=hello

# Load Modules
module purge
module load gcc/5.4.0
module load openmpi/3.0.0

echo "SLURM_NTASKS = " $SLURM_NTASKS

# Execute using standard mpirun
mpirun -n $SLURM_NTASKS hello_world.X >& hello_world_gnu.out

# Alternative: Execute using SLURM's srun
# srun --mpi=pmi2 hello_world.X >& hello_world_gnu.out
