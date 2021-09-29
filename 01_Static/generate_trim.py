import aircraft
import numpy as np
import sharpy.utils.algebra as algebra


def generate_trim(alpha=0., elevator=0., thrust=0., dt=0.1, case_name='hale_trim', case_route='./', **kwargs):

    output_route = kwargs.get('output_route', './output/')
    m = kwargs.get('M', 4)

    hale = aircraft.Hale(case_name, case_route, output_route)

    hale.clean()

    hale.init_structure(**kwargs)
    hale.init_aero(m, **kwargs)

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
                                 'relaxation_factor': kwargs.get('relaxation_factor', 0.3)}
    
    if kwargs.get('polars', None) is not None:
        settings['StaticCoupled']['correct_forces_method'] = 'PolarCorrection'
        settings['StaticCoupled']['correct_forces_settings'] = {'cd_from_cl': 'off',
                                                               'correct_lift': 'on',
                                                               'moment_from_polar': 'on'}

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

    settings['AeroForcesCalculator'] = {'write_text_file': 'on'}

    hale.create_settings(settings)
    
def load_airfoil_tools_polars(filename):
    """CSV file from airfoil tools"""
    
    data = np.loadtxt(filename, skiprows=11, delimiter=',')
    
    aoa_rad = data[:, 0] * np.pi / 180
    cl = data[:, 1]
    cd = data[:, 2]
    cm = data[:, 4]

    return np.column_stack((aoa_rad, cl, cd, cm))

def main():
    # Flight conditions
    u_inf = 8
    alpha = 4 * np.pi/180
    cs_elevator = 1 * np.pi/180
    thrust = 5
    use_polar = False

    # Discretisation
    M = 4
    n_elem_multiplier = 1.5
    wake_length = 10
    horseshoe = True
    dt = 1/M/u_inf

    flow = [
        'BeamLoader',
        'AerogridLoader',
        # 'NonLinearStatic',
        # 'StaticUvlm',
        'StaticTrim',
#         'StaticCoupled',
        # 'Modal',
        # 'BeamLoads',
        'AerogridPlot',
        'BeamPlot',
        'AeroForcesCalculator',
        # 'SaveData',
    ]
    
    if use_polar:
        polars = load_airfoil_tools_polars('./xf-naca0018-il-50000.csv')
    else:
        polars = None

    generate_trim(alpha=alpha,
                  elevator=cs_elevator,
                  thrust=thrust,
                  dt=dt, case_name='hale_trim_polar{:g}'.format(use_polar), case_route='./cases/',
                    flow=flow,
                    u_inf=u_inf,
                    M=M,
                    sigma=1.5,
                    n_elem_multiplier=n_elem_multiplier,
                    horseshoe=horseshoe,
                    wake_length=wake_length,
                    relaxation_factor=0.6,
                    tolerance=1e-5,
                    fsi_tolerance=1e-5,
                    polars=polars)


if __name__ == '__main__':
    main()

