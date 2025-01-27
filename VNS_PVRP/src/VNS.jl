module VNS

using ..PVRPInstance: PVRPInstanceStruct
using ..Solution: PVRPSolution, VRPSolution, Route, recalculate_route!, remove_segment!, insert_segment!, validate_solution, display_solution, plot_solution, save_solution_to_yaml, save_run_info_to_yaml, VNSLogbook, initialize_logbook, update_logbook!, save_logbook_to_yaml, plot_logbook, recalculate_plan_length!, run_parameter_study
using ..ConstructionHeuristics: nearest_neighbor
using ..LocalSearch: local_search!
using ..Shaking: shaking!, change_visit_combinations!
using Random
using FilePathsBase: mkpath, joinpath
using Dates: now
using Plots: savefig  # Import savefig from Plots

export vns!, test_vns!

function vns!(solution::PVRPSolution, instance::PVRPInstanceStruct, instance_name::String, num_iterations::Int, save_folder::String, seed::Int)::Tuple{PVRPSolution, VNSLogbook}
    # Set the random seed for reproducibility
    Random.seed!(seed)

    # Initialize variables
    logbook = initialize_logbook()
    best_solution = deepcopy(solution)
    best_feasible_solution = deepcopy(solution)

    for iteration in 1:num_iterations
        try
            # Shaking using only `changevisitcombinations!`
            change_visit_combinations!(solution, instance)

            # Perform local search on all changed routes
            for day in keys(solution.tourplan)
                for route in solution.tourplan[day].routes
                    if route.changed
                        local_search!(route, instance, "2opt-first", 1000)
                        route.changed = false
                    end
                end
            end

            # Recalculate the total plan length and duration
            recalculate_plan_length!(solution)

            # Align initial visit combinations with the current state
            for day in keys(solution.tourplan)
                for route in solution.tourplan[day].routes
                    for node in route.visited_nodes
                        if node != 0
                            instance.nodes[node + 1].initialvisitcombination[day] = true
                        end
                    end
                end
            end

            # Validate and update the best solutions
            is_feasible = validate_solution(solution, instance)
            if is_feasible && solution.plan_length < best_feasible_solution.plan_length
                best_feasible_solution = deepcopy(solution)
            end

            if solution.plan_length < best_solution.plan_length
                best_solution = deepcopy(solution)
                println("New best solution found at iteration $iteration: $(best_solution.plan_length)")
            end

            # Log current state
            update_logbook!(
                logbook,
                iteration,
                solution.plan_length,
                best_solution.plan_length,
                best_feasible_solution.plan_length,
                feasible=is_feasible
            )

        catch e
            println("Error during iteration $iteration: $e")
            continue
        end

        # Prepare for next iteration by using the best solution so far
        solution = deepcopy(best_solution)
    end

    # Create the necessary directories
    instance_folder = joinpath(save_folder, instance_name)
    seed_folder = joinpath(instance_folder, string(seed))
    mkpath(seed_folder)

    # Save logbook and best solution
    save_solution_to_yaml(best_solution, joinpath(seed_folder, "solution.yaml"))
    save_logbook_to_yaml(logbook, joinpath(seed_folder, "logbook.yaml"))

    # Save run information
    save_run_info_to_yaml(seed, 0.0, best_solution.plan_length, validate_solution(best_solution, instance), joinpath(seed_folder, "run_info.yaml"))

    # Save the solution evolution plot
    solution_plot_logbook = plot_logbook(logbook, instance_name, seed, seed_folder)
    savefig(solution_plot_logbook, joinpath(seed_folder, "solution_evolution_plot.png"))

    # Save the solution plot
    solution_plot = plot_solution(best_solution, instance)
    savefig(solution_plot, joinpath(seed_folder, "solution_plot.png"))

    # Run parameter study and save results
    # run_parameter_study(instance, best_solution, seed_folder)

    return best_solution, logbook
end

function test_vns!(instance::PVRPInstanceStruct, instance_name::String, num_runs::Int, save_folder::String, num_iterations::Int)
    results = []

    # Generate a seed for the run
    seed = rand(1:10000)

    for run in 1:num_runs
        # Reset the instance for each run
        instance_copy = deepcopy(instance)

        # Initialize with nearest neighbor solution
        initial_solution = nearest_neighbor(instance_copy)

        # Run VNS and collect the best solution and logbook
        best_solution, logbook = vns!(initial_solution, instance_copy, instance_name, num_iterations, save_folder, seed)

        # Align initial visit combinations with the current state
        for day in keys(best_solution.tourplan)
            for route in best_solution.tourplan[day].routes
                for node in route.visited_nodes
                     if node != 0
                        instance_copy.nodes[node + 1].initialvisitcombination[day] = true
                    end
                end
            end
        end
        is_valid = validate_solution(best_solution, instance_copy)

        push!(results, (best_solution, logbook, is_valid, seed))
    end

    return results
end

end # module
