import os
import sys
sys.path.append('/home/ng213/2TB/KK_AirbusHALE/src/analysis/')
from batch.sets import Actual
import numpy as np

sharpy_output_path = './output/'
case_name_root = 'nonlin_forces'

postproc_output = './nonlinforces.txt'

data = Actual(sharpy_output_path + '/' + case_name_root + '*')
data.load_bulk_cases('forces')

np.savetxt(postproc_output, np.column_stack(data.forces()))