import aircraft
import numpy as np
import sharpy.utils.algebra as algebra


def generate_derivatives(alpha=0., elevator=0., thrust=0., dt=0.1, case_name='hale_static', case_route='./', **kwargs):

    output_route = kwargs.get('output_route', './output/')
    m = kwargs.get('M', 4)
    rho = kwargs.get('rho', 1.225)
    tolerance = kwargs.get('tolerance', 1e-5)

    hale = aircraft.Hale(case_name, case_route, output_route)

    hale.clean()

    hale.init_structure(**kwargs)
    hale.init_aero(m)

    hale.set_flight_controls(thrust=thrust, elevator=elevator)
    hale.generate()

    settings = dict()
    u_inf = kwargs.get('u_inf', 10)

    settings['SHARPy'] = {'case': case_name,
                          'route': case_route,
                          'flow': kwargs.get('flow', []),
                          'write_screen': 'on',
                          'write_log': 'on',
                          'log_folder': './output/',
                          'log_file': case_name + '.log'}

    settings['BeamLoader'] = {'unsteady': 'on',
                              'orientation': algebra.euler2quat(np.array([0.,
                                                                          alpha,
                                                                          0.]))}

    settings['AerogridLoader'] = {'unsteady': 'on',
                                  'aligned_grid': 'on',
                                  'mstar': int(kwargs.get('wake_length', 10) * m),
                                  'control_surface_deflection': ['', ''],
                                  'control_surface_deflection_generator_settings':
                                      {'0': {},
                                       '1': {}},
                                  'wake_shape_generator': 'StraightWake',
                                  'wake_shape_generator_input': {
                                      'u_inf': u_inf,
                                      'u_inf_direction': [1., 0., 0.],
                                      'dt': dt,
                                  },
                                  }

    settings['NonLinearStatic'] = {'print_info': 'off',
                                   'max_iterations': 150,
                                   'num_load_steps': 1,
                                   'delta_curved': 1e-1,
                                   'min_delta': tolerance,
                                   'gravity_on': kwargs.get('gravity', 'on'),
                                   'gravity': 10.6235,
                                   'initial_position': [0., 0., 0.]}

    settings['StaticUvlm'] = {'print_info': 'on',
                              'horseshoe': kwargs.get('horseshoe', 'off'),
                              'num_cores': 4,
                              'n_rollup': 0,
                              'rollup_dt': dt,
                              'rollup_aic_refresh': 1,
                              'rollup_tolerance': 1e-4,
                              'velocity_field_generator': 'SteadyVelocityField',
                              'velocity_field_input': {'u_inf': u_inf,
                                                       'u_inf_direction': [1., 0, 0]},
                              'rho': rho}

    settings['StaticCoupled'] = {'print_info': 'off',
                                 'structural_solver': 'NonLinearStatic',
                                 'structural_solver_settings': settings['NonLinearStatic'],
                                 'aero_solver': 'StaticUvlm',
                                 'aero_solver_settings': settings['StaticUvlm'],
                                 'max_iter': 100,
                                 'n_load_steps': kwargs.get('n_load_steps', 1),
                                 'tolerance': kwargs.get('fsi_tolerance', 1e-5),
                                 'relaxation_factor': kwargs.get('relaxation_factor', 0.2)}

    settings['StaticTrim'] = {'solver': 'StaticCoupled',
                              'solver_settings': settings['StaticCoupled'],
                              'initial_alpha': alpha,
                              'initial_deflection': elevator,
                              'initial_thrust': thrust,
                              'fz_tolerance': 0.1,
                              'fx_tolerance': 0.1,
                              'm_tolerance': 0.1,
                              'save_info': 'on',
                              }

    settings['BeamPlot'] = {
                            'include_FoR': 'on'}

    settings['AerogridPlot'] = {
                                'include_rbm': 'off',
                                'include_applied_forces': 'on',
                                'minus_m_star': 0,
                                'u_inf': u_inf
                                }

    settings['AeroForcesCalculator'] = {
                                        'write_text_file': 'on',
                                        'text_file_name': 'aeroforces.txt',
                                        'screen_output': 'on',
                                        'coefficients': True,
                                        'q_ref': 0.5 * rho * u_inf ** 2,
                                        'S_ref': hale.structure.span_main * hale.aero.chord_main,
                                        }

    settings['BeamPlot'] = {
                            'include_rbm': 'on',
                            'include_applied_forces': 'on',
                            'include_FoR': 'on'}

    struct_solver_settings = {'print_info': 'off',
                              'initial_velocity_direction': [-1., 0., 0.],
                              'max_iterations': 950,
                              'delta_curved': 1e-6,
                              'min_delta': tolerance,
                              'newmark_damp': 5e-3,
                              'gravity_on': kwargs.get('gravity', 'on'),
                              'gravity': 10.6235,
                              'num_steps': 1,
                              'dt': dt,
                              'initial_velocity': u_inf * 1}

    step_uvlm_settings = {'print_info': 'on',
                          'num_cores': 4,
                          'convection_scheme': 2, #ws.wake_type,
                          'vortex_radius': 1e-6,
                          'velocity_field_generator': 'SteadyVelocityField',
                          'velocity_field_input': {'u_inf': u_inf * 0,
                                                   'u_inf_direction': [1., 0., 0.]},
                          'rho': rho,
                          'n_time_steps': 1,
                          'dt': dt,
                          'gamma_dot_filtering': 3}

    settings['DynamicCoupled'] = {'print_info': 'on',
                                  # 'structural_substeps': 1,
                                  # 'dynamic_relaxation': 'on',
                                  # 'clean_up_previous_solution': 'on',
                                  'structural_solver': 'NonLinearDynamicCoupledStep',
                                  'structural_solver_settings': struct_solver_settings,
                                  'aero_solver': 'StepUvlm',
                                  'aero_solver_settings': step_uvlm_settings,
                                  'fsi_substeps': 200,
                                  'fsi_tolerance': tolerance,
                                  'relaxation_factor': 0.3,
                                  'minimum_steps': 1,
                                  'relaxation_steps': 150,
                                  'final_relaxation_factor': 0.5,
                                  'n_time_steps': 1,  # ws.n_tstep,
                                  'dt': dt,
                                  'include_unsteady_force_contribution': 'off'}
    # 'postprocessors': ['BeamPlot', 'AerogridPlot', 'WriteVariablesTime'],
    # 'postprocessors_settings': {'BeamLoads': {'folder': output_route,
    #                                           'csv_output': 'off'},
    #                             'BeamPlot': {'folder': output_route,
    #                                          'include_rbm': 'on',
    #                                          'include_applied_forces': 'on'},
    #                             'AerogridPlot': {
    #                                 'u_inf': ws.u_inf,
    #                                 'folder': output_route,
    #                                 'include_rbm': 'on',
    #                                 'include_applied_forces': 'on',
    #                                 'minus_m_star': 0},
    #                             'WriteVariablesTime': {
    #                                 'folder': output_route,
    #                                 'cleanup_old_solution': 'on',
    #                                 'delimiter': ',',
    #                                 'FoR_variables': ['total_forces',
    #                                                   'total_gravity_forces',
    #                                                   'for_pos', 'quat'],
    #                             }}}

    settings['StabilityDerivatives'] = {
                                        'u_inf': u_inf,
                                        'c_ref': hale.aero.chord_main,
                                        'b_ref': hale.structure.span_main * 2,
                                        'S_ref': 2 * hale.structure.span_main * hale.aero.chord_main,
                                        }

    settings['Modal'] = {'print_info': True,
                         'use_undamped_modes': True,
                         'NumLambda': 50,
                         'rigid_body_modes': True,
                         'write_modes_vtk': 'on',
                         'print_matrices': 'on',
                         'save_data': 'on',
                         'continuous_eigenvalues': 'off',
                         'dt': dt,
                         'plot_eigenvalues': False,
                         'rigid_modes_ppal_axes': 'off',
                         'rigid_modes_cg': 'on',
                         }
    # ROM settings
    rom_settings = dict()
    rom_settings['algorithm'] = 'mimo_rational_arnoldi'
    rom_settings['r'] = 4
    rom_settings['frequency'] = np.array([0], dtype=float)
    rom_settings['single_side'] = 'observability'

    settings['LinearAssembler'] = {'linear_system': 'LinearAeroelastic',
                                   'linear_system_settings': {
                                       'beam_settings': {'modal_projection': 'on',
                                                         'inout_coords': 'modes',
                                                         'discrete_time': 'off',
                                                         'newmark_damp': 0.5e-3,
                                                         'discr_method': 'newmark',
                                                         'dt': dt,
                                                         'proj_modes': 'undamped',
                                                         'use_euler': 'on',
                                                         'num_modes': 20,
                                                         'print_info': 'on',
                                                         'gravity': kwargs.get('gravity', 'on'),
                                                         'remove_dofs': [],
                                                         'remove_rigid_states': 'on'},
                                       'aero_settings': {'dt': dt,
                                                         # 'ScalingDict': {'length': hale.aero.chord_main / 2,
                                                         #                  'speed': u_inf,
                                                         #                  'density': rho},
                                                         'integr_order': 2,
                                                         'density': rho,
                                                         'remove_predictor': 'off',
                                                         'use_sparse': False,
                                                         'vortex_radius': 1e-7,
                                                         'remove_inputs': ['u_gust'],
                                                         'convert_to_ct': 'on',
                                                         # 'rom_method': ['Krylov'],
                                                         # 'rom_method_settings': {'Krylov': rom_settings},
                                                         },
                                       'track_body': 'off',
                                   }}

    settings['AsymptoticStability'] = {
        'print_info': 'on',
        'modes_to_plot': [],
        # 'velocity_analysis': [27, 29, 3],
        'display_root_locus': 'off',
        'frequency_cutoff': 0,
        'export_eigenvalues': 'on',
        'num_evals': 1000,
        'target_system': ['aeroelastic', 'aerodynamic', 'structural'],
        'folder': output_route}

    settings['FrequencyResponse'] = {'target_system': ['aeroelastic', 'aerodynamic', 'structural'],
                                     'quick_plot': 'off',
                                     'frequency_spacing': 'log',
                                     'frequency_unit': 'w',
                                     'frequency_bounds': [1e-3, 1e3],
                                     'num_freqs': 200,
                                     'print_info': 'on'}

    settings['PickleData'] = {}

    hale.create_settings(settings)

    return hale


def main():
    # Flight conditions
    u_inf = 6

    # for u_inf in [6, 7, 8, 9, 11, 12, 13, 14]:
    # print(u_inf)
    alpha = 0 * np.pi / 180
    elevator = 0 * np.pi / 180
    thrust = 0

    # Discretisation
    M = 4
    n_elem_multiplier = 1.5
    wake_length = 40
    horseshoe = False

    flow = [
        'BeamLoader',
        'AerogridLoader',
        # 'NonLinearStatic',
        'StaticUvlm',
        # 'StaticTrim',
        # 'StaticCoupled',
        # 'DynamicCoupled',
        'Modal',
        # 'BeamLoads',
        # 'AerogridPlot',
        # 'BeamPlot',
        'AeroForcesCalculator',
        'LinearAssembler',
        # 'AsymptoticStability',
        'PickleData',
        'StabilityDerivatives',
        # 'FrequencyResponse',
        # 'SaveData',
    ]

    # flow = ['StabilityDerivatives']
    restart = False
    # flow = ['FrequencyResponse']
    import sharpy.sharpy_main as smain

    # for exponent in [0, 1, 2, 3, 4, 5]:
    for exponent in [5]:
        # for u_inf in np.linspace(5, 15, 11):
        dt = 1/M/u_inf
        hale = generate_derivatives(alpha=alpha, dt=dt, case_name='hale_static_m{:g}_uinf{:g}_rm_ct'.format(M, int(u_inf)),
                                    case_route='./cases/',
                                    elevator=elevator,
                                    thrust=thrust,
                                    flow=flow,
                                    u_inf=u_inf,
                                    M=M,
                                    n_elem_multiplier=n_elem_multiplier,
                                    horseshoe=horseshoe,
                                    wake_length=wake_length,
                                    relaxation_factor=0.6,
                                    tolerance=1e-5,
                                    gravity='off',
                                    fsi_tolerance=1e-5,
                                    # sigma=1 * 10 ** exponent,
                                    )

        if restart:
            smain.main(['',hale.case_route + '/' + hale.case_name + '.sharpy', '-r', './output/' + hale.case_name + '.pkl'])
        else:
            smain.main(['',hale.case_route + '/' + hale.case_name + '.sharpy'])

        del hale

if __name__ == '__main__':
    main()
