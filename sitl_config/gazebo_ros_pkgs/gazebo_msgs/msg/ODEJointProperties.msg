# access to low level joint properties, change these at your own risk
float64[] damping             # joint damping
float64[] hiStop              # joint limit
float64[] loStop              # joint limit
float64[] erp                 # set joint erp
float64[] cfm                 # set joint cfm
float64[] stop_erp            # set joint erp for joint limit "contact" joint
float64[] stop_cfm            # set joint cfm for joint limit "contact" joint
float64[] fudge_factor        # joint fudge_factor applied at limits, see ODE manual for info.
float64[] fmax                # ode joint param fmax
float64[] vel                 # ode joint param vel
