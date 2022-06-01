rosservice call /gazebo/set_physics_properties "time_step: 0.1
max_update_rate: 10 
gravity:
  x: 0.0
  y: 0.0
  z: 0.0
ode_config: {auto_disable_bodies: false, sor_pgs_precon_iters: 0, sor_pgs_iters: 0,
  sor_pgs_w: 0.0, sor_pgs_rms_error_tol: 0.0, contact_surface_layer: 0.0, contact_max_correcting_vel: 0.0,
  cfm: 0.0, erp: 0.0, max_contacts: 0}"