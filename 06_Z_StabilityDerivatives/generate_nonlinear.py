import aircraft
import numpy as np
import sharpy.utils.algebra as algebra


def generate_forces(alpha=0., elevator=0., thrust=0., dt=0.1, case_name='hale_forces', case_route='./', **kwargs):

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

    settings['AerogridLoader'] = {'unsteady': 'off',
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
                                        'S_ref': 2 * hale.structure.span_main * hale.aero.chord_main,
                                        'b_ref': 2 * hale.structure.span_main
                                        }

    settings['SaveParametricCase'] = {
                                      'save_case': 'off',
                                      'parameters': {'alpha': alpha}}

    hale.create_settings(settings)

    hale.run()


def main():
    # Flight conditions
    # for M in [4, 8, 16, 32]:
    # for u_inf in np.linspace(5, 15, 11):

    sim_type = 'aerodynamic'  # or aeroelastic

    for u_inf in [6]:
        # u_inf = 10
        alpha0 = 4
        eps = 1e-6

        # 4.1516 | -0.4894 | 5.3966
        alpha0 = 4.1516 * np.pi/180
        elevator = -0.4894 * np.pi/180
        thrust = 5.3966

        alpha0 = 0
        elevator = 0
        thrust = 0

        # Discretisation
        M = 4
        n_elem_multiplier = 4 #1.5
        wake_length = 40
        horseshoe = False
        dt = 1 / M / u_inf

        if sim_type == 'aerodynamic':
            aero_solver = 'StaticUvlm'
        elif sim_type == 'aeroelastic':
            aero_solver = 'StaticCoupled'

        flow = [
            'BeamLoader',
            'AerogridLoader',
            aero_solver,
            'AeroForcesCalculator',
            'SaveParametricCase'
        ]

        # alpha_vec = np.linspace(alpha0 * (1 - eps), alpha0 * (1 + eps), 6)
        alpha_vec = np.linspace(- eps, + eps, 6)
        for ith, alpha in enumerate(alpha_vec):
            case_name = 'zero_m{:g}_nonlin_{:s}_forces_u{:04g}_aind{:02g}'.format(M, sim_type, u_inf * 10, ith)
            generate_forces(alpha=alpha, dt=dt, case_name=case_name, case_route='./cases/',
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
                            gravity='on',
                            # sigma=1e5,
                            fsi_tolerance=1e-5)


if __name__ == '__main__':
    main()
