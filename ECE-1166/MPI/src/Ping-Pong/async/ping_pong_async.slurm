#!/bin/bash
#SBATCH --job-name=mattis_ping_pong_async
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
mpirun -n $SLURM_NTASKS ./ping_pong_async.x 10 &>> ping_pong_async.out
mpirun -n $SLURM_NTASKS ./ping_pong_async.x 100 &>> ping_pong_async.out
mpirun -n $SLURM_NTASKS ./ping_pong_async.x 200 &>> ping_pong_async.out
mpirun -n $SLURM_NTASKS ./ping_pong_async.x 800 &>> ping_pong_async.out
mpirun -n $SLURM_NTASKS ./ping_pong_async.x 1400 &>> ping_pong_async.out
mpirun -n $SLURM_NTASKS ./ping_pong_async.x 2000 &>> ping_pong_async.out
mpirun -n $SLURM_NTASKS ./ping_pong_async.x 4000 &>> ping_pong_async.out
mpirun -n $SLURM_NTASKS ./ping_pong_async.x 8000 &>> ping_pong_async.out
mpirun -n $SLURM_NTASKS ./ping_pong_async.x 10000 &>> ping_pong_async.out

# Alternative: Execute using SLURM's srun
# srun --mpi=pmi2 ping_pong_async.x [data_size] >& ping_pong_async.out
