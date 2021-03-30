import ZachsModules as zm
import sys
from datetime import datetime as dt
import os




def multiTasker(Ncases, Nnodes, verbose=False):
    N_sect = int(Ncases/Nnodes)
    N_leftover = Ncases % Nnodes
    terminal = 1
    start = 1
    count = 0
    cases = [None] * Nnodes
    for i in range(1,Ncases+1):
        count += 1
        if N_leftover > 0:
            if count >= N_sect + 1:
                cases[terminal-1] = [terminal,start,i]
                count = 0
                N_leftover -= 1
                start = i+1
                terminal += 1
        else:
            if count >= N_sect:
                cases[terminal-1] = [terminal,start,i]
                count = 0
                terminal += 1
                start = i+1
    if verbose: displayCases(cases)
    return cases

def displayCases(C):
    for c in C:
        t,s,e = c
        print('Slurm {:1d}: start {:2d}, end {:2d}'.format(t,s,e))


lines = [   '#!/bin/bash',
            '#SBATCH --time=5-00:00:00',
            '#SBATCH --nodes=1',
            '#SBATCH --account=usumae-kp',
            '#SBATCH --partition=usumae-kp',
            #'#SBATCH -C c64',
            '#SBATCH -o GenAeroData-%j',
            '#SBATCH --mail-user=zachary.s.montgomery@gmail.com',
            '#SBATCH --mail-type=END',
            'module purge',
            'module load python/3.7.3',
            'cd /uufs/chpc.utah.edu/common/home/u6035531/usuAeroLabCodes/Horizon-Flying-Wing',
            'python GenAeroData.py {} {} {} {}']


if __name__ == '__main__':
    Ncases = int(eval(sys.argv[1]))
    Nnodes = int(sys.argv[2])
    
    slurms = multiTasker(Ncases,Nnodes)
    
    for slurm in slurms:
        
        jid = str(dt.now()).replace(':','_').replace(' ','__')
        
        start, end = slurm[1:]
        
        ncases = end - start
        if ncases <= 1e6:
            batch = ncases
        else:
            batch = 1e6
        chunck = 3
        
        lines[-1].format(start, end, batch, chunck)
        lines[-1] += ' > out_$SLURM_JOB_ID.txt'
        
        fn = 'job_{}.slurm'.format(jid)
        f = open(fn, 'w')
        f.write('\n'.join(lines)+'\n')
        f.close()
        
        os.system('sbatch {}'.format(fn))
        
        displayCases((slurm,))






