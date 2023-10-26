# tilting_drone_jerk
## How to execute
1. Generate the .cpp functions by executing "drone_model_sym" and "controller_sym" (only once or at each change in the models)
2. Launch "model_simu" to simulate and show the plots

## List of file:
- "drone_model_sym" : define the symbolic model of the drone
- "controller_sym" : define the symbolic model of the controller
- "drone_settings" : contains the parameters of the drone (used to generate the models)
- "params" : includes "drone_settings" and add also the controller gains (used to simulate)
- "model_simu" : simulate the controlled system with a desired trajectory (defined with Bezier curve)
- "single_sim" : contains the simulation of the model (is used by "model_simu"
- "jerk_matrix" : evaluate the augumented allocation matrix, given the drone state
- "show_jerk_ellips" : plot the jerk ellipsoid

