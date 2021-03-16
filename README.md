# IROS17+Journal MATLAB Files.

New Informative Path Planning for Project 'Flourish' based on continuous maps and Gaussian Processes.

* Marija Popovic
* Teresa Vidal-Calleja

***
## Folders in this repo:

### Not so useful to look at

* `dev`: Stand-alone examples that were useful during the development of the algorithm.
* `old`: Even older stuff. Don't go there.
* `lib`: Libraries - CMA-ES, GPs, MAV polynomial planning.
* `ROS`: MATLAB-ROS interface used for IROS17 indoor experiments in Vicon area.
* `other`: Things relating to IW Bonn + ROSBOOK.

### Useful to look at

* `tools`: All the functions called by the main programs.
* `planning_final`: Final programs of the implementation.
* `evals`: Benchmarks, simulation trial scripts, simulation results,
* `journal`: Stuff and investigations specific to journal. WIP.


## Example

To run the final planning program, execute the following steps in your MATLAB command window:

```commandline
addpath(genpath('/path/to/your/repo/'))
[matlab_params, planning_params, opt_params, map_params] = load_params(50*0.75, 50*0.75, true)
ground_truth_map = create_continuous_map(50, 50, 2, true)
[metrics, grid_map] = GP_iros(matlab_params, planning_params, opt_params, map_params, ground_truth_map)
```