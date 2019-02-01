#!/bin/bash
#SBATCH --job-name=mattis_mmult
#SBATCH --nodes=2
#SBATCH --ntasks-per-node=1
#SBATCH --cluster=mpi
#SBATCH --mail-user=zachary.mattis@pitt.edu
#SBATCH --mail-type=FAIL
#SBATCH --time=0-00:2:00
#SBATCH --qos=short

# Load Modules
module purge
module load intel/2017.3.196

# Env Variables
echo "SLURM_JOB_NAME = " $SLURM_JOB_NAME
echo "SLURM_NTASKS = " $SLURM_NTASKS
echo "SLURM_JOB_ID = " $SLURM_JOB_ID

# Execute using standard mpirun
mpirun -n $SLURM_NTASKS ./mmult.x &>> mmult.out

# Alternative: Execute using SLURM's srun
# srun --mpi=pmi2 mmult.x >& mmult.out
