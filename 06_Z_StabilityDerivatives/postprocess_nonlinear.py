import os
import sys
sys.path.append('/home/ng213/2TB/KK_AirbusHALE/src/analysis/')
from batch.sets import Actual
import numpy as np
import glob
import configobj

sharpy_output_path = './output/'
postproc_output_folder = './nonlinear_output/'
sim_type = 'aerodynamic'

if not os.path.isdir(postproc_output_folder):
    os.makedirs(postproc_output_folder)

u_inf_vec = np.linspace(5, 15, 11)
u_inf_vec = [6]
M = 4
for u_inf in u_inf_vec:
    # for M in [4, 8, 16, 32]:
    case_name_root = 'zero_m{:g}_nonlin_{:s}_forces_u{:04g}'.format(M, sim_type, u_inf * 10)

    postproc_output = postproc_output_folder +'/zero_m{:g}_nonlinforces_{:s}_u{:04g}.txt'.format(M, sim_type, u_inf * 10)

    # data = Actual(sharpy_output_path + '/' + case_name_root + '*')
    # data.load_bulk_cases('forces')
    #
    # np.savetxt(postproc_output, np.column_stack(data.forces()))
    # np.savetxt(postproc_output.replace('forces', 'moments'), np.column_stack(data.moments()))

    source_cases = glob.glob(sharpy_output_path + '/' + case_name_root + '*')
    alpha_vec = []
    forces_vec = []
    moments_vec = []
    for case in source_cases:
        pmor_dict = configobj.ConfigObj(case + '/' + case.split('/')[-1] + '.pmor.sharpy')

        case_forces = np.loadtxt(pmor_dict['sim_info']['path_to_data'] + '/forces/forces_aeroforces.txt', skiprows=1,
                                 delimiter=',')[1:4]
        case_moments = np.loadtxt(pmor_dict['sim_info']['path_to_data'] + '/forces/moments_aeroforces.txt',
                                  skiprows=1, delimiter=',')[1:4]

        alpha_vec.append(float(pmor_dict['parameters']['alpha']))
        forces_vec.append(case_forces)
        moments_vec.append(case_moments)

    order = np.argsort(np.array(alpha_vec))
    alpha_vec = np.array([alpha_vec[ith] for ith in order])
    forces_vec = np.vstack(([forces_vec[ith] for ith in order]))
    moments_vec = np.vstack(([moments_vec[ith] for ith in order]))

    np.savetxt(postproc_output, np.column_stack((alpha_vec, forces_vec)))
    np.savetxt(postproc_output.replace('forces', 'moments'), np.column_stack((alpha_vec, moments_vec)))

