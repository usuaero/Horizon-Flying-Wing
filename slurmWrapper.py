import ZachsModules as zm
import sys
from datetime import datetime as dt
import os




def multiTasker(Ncases, Nnodes, verbose=False):
    N_sect = int(Ncases/Nnodes)
    
    cases = [None]*Nnodes
    
    amounts = [N_sect] * Nnodes
    
    step = 1
    if sum(amounts) > Ncases: step = -1
    
    i = -1
    while sum(amounts) != Ncases:
        
        i += 1
        if i >= Nnodes: i = 0
        
        amounts[i] += step
    
    cnt = 1
    
    for i in range(Nnodes):
        cases[i] = (i+1, cnt, cnt + amounts[i] - 1)
        cnt += amounts[i]
    
    if verbose: displayCases(cases)
    return cases

def displayCases(C):
    print(('{:^5}'+'  {:^20}'*3).format('Slurm', 'Start', 'End', 'Length'))
    print(('{:=^5}'+'  {:=^20}'*3).format('', '', '', ''))
    
    for c in C:
        t,s,e = c
        print(('{:^5d}'+'  {:>20d}'*3).format(t,s,e,e-s+1))


template = ['#!/bin/bash',
            '#SBATCH --time=5-00:00:00',
            '#SBATCH --nodes=1',
            '#SBATCH --account=usumae-kp',
            '#SBATCH --partition=usumae-kp',
            #'#SBATCH -C c64',
            '#SBATCH -o LinearAeroData-%j',
            '#SBATCH --mail-user=zachary.s.montgomery@gmail.com',
            '#SBATCH --mail-type=ALL',
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
        batch = int(ncases / 4)
        chunck = int(batch / 64 / 4)
        if chunck < 1: chunk = 1
        
        lines = template[:]
        lines[-1] = lines[-1].format(start, end, batch, chunck)
        lines[-1] += ' > out{}.txt'.format(slurm[0])
        
        fn = 'linear{}.slurm'.format(slurm[0])
        f = open(fn, 'w')
        f.write('\n'.join(lines)+'\n')
        f.close()
        
        os.system('sbatch {}'.format(fn))
        
    displayCases(slurms)
    
    
    # multiTasker(Ncases, Nnodes, verbose=True)






