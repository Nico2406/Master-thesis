module VNS

using ..PVRPInstance: PVRPInstanceStruct
using ..Solution: PVRPSolution, VRPSolution, Route, recalculate_route!, remove_segment!, insert_segment!, validate_solution, display_solution, plot_solution, save_solution_to_yaml, save_run_info_to_yaml, VNSLogbook, initialize_logbook, update_logbook!, save_logbook_to_yaml, plot_logbook, recalculate_plan_length!, run_parameter_study, load_solution_from_yaml
using ..ConstructionHeuristics: nearest_neighbor
using ..LocalSearch: local_search!
using ..Shaking: shaking!, change_visit_combinations!, move!, change_visit_combinations_sequences!, change_visit_combinations_sequences_no_improvement!
using Random
using FilePathsBase: mkpath, joinpath
using Dates: now
using Plots: savefig  # Import savefig from Plots

export vns!, test_vns!, optimize_loaded_solution!

function vns!(solution::PVRPSolution, instance::PVRPInstanceStruct, instance_name::String, num_iterations::Int, save_folder::String, seed::Int, acceptance_probability::Float64, acceptance_iterations::Int, no_improvement_iterations::Int)::Tuple{PVRPSolution, VNSLogbook}
    # Set the random seed for reproducibility
    Random.seed!(seed)

    # Initialize variables
    logbook = initialize_logbook()
    best_solution = deepcopy(solution)
    best_feasible_solution = deepcopy(solution)
    current_solution = deepcopy(solution)
    last_accepted_iteration = 0
    last_improvement_iteration = 0
    last_worse_solution_accepted_iteration = -acceptance_iterations  # Initialize to allow acceptance at the start

    # Create the necessary directories
    instance_folder = joinpath(save_folder, instance_name)
    seed_folder = joinpath(instance_folder, string(seed))
    mkpath(seed_folder)

    is_feasible = false  # Initialize is_feasible variable

    iteration = 1
    while iteration <= num_iterations
        try
            # Start with the best known solution so far
            current_solution = deepcopy(best_solution)

            # Systematic neighborhood search (k starts at 1)
            k = 1
            while k <= 15  # 15 defined neighborhoods
                
                # Shaking with the current neighborhood k
                shaking!(current_solution, instance, k)
                
                # Perform local search on all changed routes
                for day in keys(current_solution.tourplan)
                    current_solution.tourplan[day].routes = filter(route -> !(isempty(route.visited_nodes) || route.visited_nodes == [0, 0]), current_solution.tourplan[day].routes)
                    for route in current_solution.tourplan[day].routes
                        if route.changed
                            local_search!(route, instance, "reinsert-first", 1000)
                            local_search!(route, instance, "swap-first", 1000)
                            local_search!(route, instance, "2opt-first", 1000)
                            recalculate_route!(route, instance)
                            route.changed = false
                        end
                    end
                end

                # Recalculate the total plan length and duration
                recalculate_plan_length!(current_solution)

                # Align initial visit combinations with the current state
                for day in keys(current_solution.tourplan)
                    for route in current_solution.tourplan[day].routes
                        for node in route.visited_nodes
                            if node != 0
                                instance.nodes[node + 1].initialvisitcombination[day] = true
                            end
                        end
                    end
                end

                # Validate and update the best solutions
                is_feasible = validate_solution(current_solution, instance)
                if is_feasible && current_solution.plan_length < best_feasible_solution.plan_length
                    best_feasible_solution = deepcopy(current_solution)
                end

                if current_solution.plan_length < best_solution.plan_length
                    best_solution = deepcopy(current_solution)
                    println("New best solution found at iteration $iteration: $(best_solution.plan_length)")
                    last_accepted_iteration = iteration
                    last_improvement_iteration = iteration
                    k = 1  # Reset to the first neighborhood
                elseif current_solution.plan_length > best_solution.plan_length &&
                       iteration - last_accepted_iteration <= acceptance_iterations &&
                       rand() < acceptance_probability &&
                       iteration - last_worse_solution_accepted_iteration > acceptance_iterations
                    println("Accepted worse solution at iteration $iteration: $(current_solution.plan_length)")
                    last_accepted_iteration = iteration
                    last_worse_solution_accepted_iteration = iteration
                    k = 1  # Reset to the first neighborhood
                else
                    k += 1  # Move to the next neighborhood
                end
            end

            # Log current state
            update_logbook!(
                logbook,
                iteration,
                current_solution.plan_length,
                best_solution.plan_length,
                best_feasible_solution.plan_length,
                feasible=is_feasible
            )

            # Save the solution and plot every 100 iterations
            if iteration % 100 == 0
                current_solution = deepcopy(best_solution)  # Update current solution with the best solution of the iteration
                save_solution_to_yaml(best_solution, joinpath(seed_folder, "solution_$iteration.yaml"))
                best_solution_plot = plot_solution(best_solution, instance)
                savefig(best_solution_plot, joinpath(seed_folder, "best_solution_plot_$iteration.png"))
                best_feasible_solution_plot = plot_solution(best_feasible_solution, instance)
                savefig(best_feasible_solution_plot, joinpath(seed_folder, "best_feasible_solution_plot_$iteration.png"))
            end

            iteration += 1

        catch e
            println("Error during iteration $iteration: $e")
            continue
        end
    end

    # Align initial visit combinations with the current state before final validation
    for day in keys(best_solution.tourplan)
        for route in best_solution.tourplan[day].routes
            for node in route.visited_nodes
                if node != 0
                    instance.nodes[node + 1].initialvisitcombination[day] = true
                end
            end
        end
    end

    # Save final solution
    save_solution_to_yaml(best_solution, joinpath(seed_folder, "final_solution.yaml"))

    # Save logbook and best solution
    save_logbook_to_yaml(logbook, joinpath(seed_folder, "logbook.yaml"))

    # Validate the final solution and store the result
    is_final_solution_valid = validate_solution(best_solution, instance)

    # Save run information
    save_run_info_to_yaml(seed, 0.0, best_solution.plan_length, is_final_solution_valid, joinpath(seed_folder, "run_info.yaml"))

    # Save the solution evolution plot
    solution_plot_logbook = plot_logbook(logbook, instance_name, seed, seed_folder)
    savefig(solution_plot_logbook, joinpath(seed_folder, "solution_evolution_plot.png"))

    # Save the solution plot
    solution_plot = plot_solution(best_feasible_solution, instance)
    savefig(solution_plot, joinpath(seed_folder, "solution_plot.png"))

    return best_solution, logbook
end

function test_vns!(instance::PVRPInstanceStruct, instance_name::String, num_runs::Int, save_folder::String, num_iterations::Int, acceptance_probability::Float64, acceptance_iterations::Int, no_improvement_iterations::Int)::Vector{Tuple{PVRPSolution, VNSLogbook, Bool, Int}}
    results = []

    for run in 1:num_runs

        # Generate a seed for the run
        seed = 3491

        # Reset the instance for each run
        instance_copy = deepcopy(instance)

        # Initialize with nearest neighbor solution
        initial_solution = nearest_neighbor(instance_copy)

        # Run VNS and collect the best solution and logbook
        best_solution, logbook = vns!(initial_solution, instance_copy, instance_name, num_iterations, save_folder, seed, acceptance_probability, acceptance_iterations, no_improvement_iterations)

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

function optimize_loaded_solution!(filepath::String, instance::PVRPInstanceStruct, instance_name::String, num_runs::Int, save_folder::String, num_iterations::Int, acceptance_probability::Float64, acceptance_iterations::Int, no_improvement_iterations::Int)::Vector{Tuple{PVRPSolution, VNSLogbook, Bool, Int}}
    results = []

    for run in 1:num_runs
        # Generate a seed for the run
        seed = rand(1:10000)

        # Reset the instance for each run
        instance_copy = deepcopy(instance)

        # Load the solution from the YAML file
        initial_solution = load_solution_from_yaml(filepath)

        # Run VNS and collect the best solution and logbook
        best_solution, logbook = vns!(initial_solution, instance_copy, instance_name, num_iterations, save_folder, seed, acceptance_probability, acceptance_iterations, no_improvement_iterations)

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
