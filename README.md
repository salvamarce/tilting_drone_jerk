# tilting_drone_jerk
## How to execute
1. Generate all the .cpp functions by executing "generate_models" (only once or at each change in the models)
2. Launch "sim_plot" to simulate and plot the results of a single simulation (without perturbations).
   Here the drone is simulated with the jerk model. Run "new_sim_plot" if you want to use the force model for the drone instead.
3. Launch "perturbation_effects" to simulate and plot the results of two simulations, one nominal and the other perturbed. 
   Here the drone is simulated with the jerk model. Run "new_perturbation_effects" if you want to use the force model for the drone instead.

## List of file:
- "drone_model_sym" : define the symbolic jerk model of the drone
- "new_drone_model_sym" : define the symbolic force model of the drone
- "controller_sym" : define the symbolic model of the controller
- "closed_loop_sens" : define the symbolic function of the sensitivity with the jerk model (the one with the force model is not ready yet)
- "drone_settings" : contains the parameters of the drone (used to generate the models)
- "parameters" : includes "drone_settings" and add also the controller gains (used to simulate)
- "system_simulation" : contains the simulation of the jerk model (is used by "sim_plot")
- "new_system_simulation" : contains the simulation of the force model (is used by "sim_plot")
- "jerk_matrix" : evaluate the augumented allocation matrix, given the drone state
- "show_jerk_ellips" : plot the jerk ellipsoid

