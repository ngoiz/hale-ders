import aircraft
import numpy as np
import sharpy.utils.algebra as algebra


def generate_static(alpha=0., elevator=0., thrust=0., dt=0.1, case_name='hale_static', case_route='./', **kwargs):

    output_route = kwargs.get('output_route', './output/')
    m = kwargs.get('M', 4)

    hale = aircraft.Hale(case_name, case_route)

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
                          'log_folder': './output/' + case_name + '/',
                          'log_file': case_name + '.log'}

    settings['BeamLoader'] = {'unsteady': 'on',
                              'orientation': algebra.euler2quat(np.array([0.,
                                                                          alpha,
                                                                          0.]))}

    settings['AerogridLoader'] = {'unsteady': 'on',
                                  'aligned_grid': 'on',
                                  'mstar': int(kwargs.get('wake_length', 10) * m),
                                  'control_surface_deflection': ['', ''],
                                  'control_surface_deflection_generator':
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
                                   'min_delta': kwargs.get('tolerance', 1e-5),
                                   'gravity_on': kwargs.get('gravity', 'on'),
                                   'gravity': 9.81,
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
                              'rho': kwargs.get('rho', 1.225)}

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
                              'folder': output_route,
                              }

    settings['BeamPlot'] = {'folder': output_route,
                            'include_FoR': 'on'}

    hale.create_settings(settings)


def main():
    # Flight conditions
    u_inf = 10
    alpha = 4

    # Discretisation
    M = 4
    n_elem_multiplier = 1.5
    wake_length = 10
    horseshoe = False
    dt = 1/M/u_inf

    flow = [
        'BeamLoader',
        # 'AerogridLoader',
        # 'NonLinearStatic',
        # 'StaticUvlm',
        # 'StaticTrim',
        # 'StaticCoupled',
        # 'Modal',
        # 'BeamLoads',
        # 'AerogridPlot',
        'BeamPlot',
        # 'AeroForcesCalculator',
        # 'SaveData',
    ]

    generate_static(alpha=alpha, dt=dt, case_name='hale_static', case_route='./cases/',
                    flow=flow,
                    u_inf=u_inf,
                    M=M,
                    n_elem_multiplier=n_elem_multiplier,
                    horseshoe=horseshoe,
                    wake_length=wake_length,
                    relaxation_factor=0.6,
                    tolerance=1e-5,
                    fsi_tolerance=1e-5)


if __name__ == '__main__':
    main()
