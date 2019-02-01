#!/bin/bash
#SBATCH --job-name=mattis_ping_pong_sync
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
mpirun -n $SLURM_NTASKS ./ping_pong_sync.x 10 &>> ping_pong_sync.out
mpirun -n $SLURM_NTASKS ./ping_pong_sync.x 100 &>> ping_pong_sync.out
mpirun -n $SLURM_NTASKS ./ping_pong_sync.x 200 &>> ping_pong_sync.out
mpirun -n $SLURM_NTASKS ./ping_pong_sync.x 800 &>> ping_pong_sync.out
mpirun -n $SLURM_NTASKS ./ping_pong_sync.x 1400 &>> ping_pong_sync.out
mpirun -n $SLURM_NTASKS ./ping_pong_sync.x 2000 &>> ping_pong_sync.out
mpirun -n $SLURM_NTASKS ./ping_pong_sync.x 4000 &>> ping_pong_sync.out
mpirun -n $SLURM_NTASKS ./ping_pong_sync.x 8000 &>> ping_pong_sync.out
mpirun -n $SLURM_NTASKS ./ping_pong_sync.x 10000 &>> ping_pong_sync.out

# Alternative: Execute using SLURM's srun
# srun --mpi=pmi2 ping_pong_sync.x [data_size] >& ping_pong_sync.out
