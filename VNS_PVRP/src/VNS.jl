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

function vns!(
    solution::PVRPSolution, 
    instance::PVRPInstanceStruct, 
    instance_name::String, 
    num_iterations::Int, 
    save_folder::String, 
    seed::Int, 
    acceptance_probability::Float64, 
    acceptance_iterations::Int, 
    no_improvement_iterations::Int
)::Tuple{PVRPSolution, VNSLogbook}
    
    # Set the random seed for reproducibility
    Random.seed!(seed)

    # Initialize variables
    logbook = initialize_logbook()
    best_solution = deepcopy(solution)
    best_feasible_solution = deepcopy(solution)
    current_solution = deepcopy(solution)
    
    last_accepted_iteration = 0
    last_improvement_iteration = 0
    last_worse_solution_accepted_iteration = -acceptance_iterations
    no_improvement_count = 0  # Counter for no improvement

    # Create directories for saving results
    instance_folder = joinpath(save_folder, instance_name)
    seed_folder = joinpath(instance_folder, string(seed))
    mkpath(seed_folder)

    iteration = 1
    k = 1  # Start with the first neighborhood
    is_feasible = false
    while iteration <= num_iterations
        try
            # Start with the best known solution unless no improvement for a while
            if no_improvement_count < no_improvement_iterations
                current_solution = deepcopy(best_solution)
            else
                println("Applying change_visit_combinations_sequences_no_improvement due to no improvement for $no_improvement_iterations iterations")
                change_visit_combinations_sequences_no_improvement!(current_solution, instance)
                no_improvement_count = 0  # Reset counter
                last_improvement_iteration = iteration
                last_accepted_iteration = iteration - acceptance_iterations  # Prevent reverting to best solution for acceptance_iterations
            end
            best_iteration_solution = deepcopy(best_solution)

            # Shaking with neighborhood k
            shaking!(current_solution, instance, k)
            
            # Apply local search on modified routes
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

            # Recalculate total plan length and duration
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

            # Validate and update best solutions
            is_feasible = all(route -> route.feasible, [route for vrp_solution in values(current_solution.tourplan) for route in vrp_solution.routes])
            if is_feasible && current_solution.plan_length < best_feasible_solution.plan_length 
                best_feasible_solution = deepcopy(current_solution)
            end

            # validate the solution
            validate_solution(current_solution, instance)

            if current_solution.plan_length < best_solution.plan_length
                best_solution = deepcopy(current_solution)
                best_iteration_solution = deepcopy(current_solution)
                println("New best solution found at iteration $iteration: $(best_solution.plan_length)")
                last_accepted_iteration = iteration
                last_improvement_iteration = iteration
                no_improvement_count = 0  # Reset counter
                k = 1  # Reset to the first neighborhood
            elseif current_solution.plan_length > best_solution.plan_length &&
                   iteration - last_accepted_iteration <= acceptance_iterations &&
                   rand() < acceptance_probability &&
                   iteration - last_worse_solution_accepted_iteration > acceptance_iterations &&
                   current_solution.plan_length <= 1.05 * best_solution.plan_length  # Accept only if within 5% worse
                #println("Accepted worse solution at iteration $iteration: $(current_solution.plan_length)")
                last_accepted_iteration = iteration
                last_worse_solution_accepted_iteration = iteration
                best_iteration_solution = deepcopy(current_solution)
                k = 1  # Reset to the first neighborhood
            else
                no_improvement_count += 1  # Increment no improvement counter
                k += 1  # Move to the next neighborhood
                if k > 15  # Reset neighborhood if it exceeds the limit
                    k = 1
                end
            end
            
            # **Update logbook inside the loop to track neighborhood-wise changes**
            update_logbook!(
                logbook,
                iteration,
                current_solution.plan_length,
                best_solution.plan_length,
                best_feasible_solution.plan_length,
                feasible=is_feasible
            )
            catch e
                println("Error during iteration $iteration: $e")
                continue
            end

            iteration += 1

        end

            # Save solution, plots, and logbook after all iterations
            save_solution_to_yaml(best_solution, joinpath(seed_folder, "solution_final.yaml"))
            best_solution_plot = plot_solution(best_solution, instance)
            savefig(best_solution_plot, joinpath(seed_folder, "best_solution_plot_final.png"))
            best_feasible_solution_plot = plot_solution(best_feasible_solution, instance)
            savefig(best_feasible_solution_plot, joinpath(seed_folder, "best_feasible_solution_plot_final.png"))
            save_logbook_to_yaml(logbook, joinpath(seed_folder, "logbook_final.yaml"))
            
            # Save the solution evolution plot
            sols_p = plot_logbook(logbook, instance_name, seed, seed_folder)
            savefig(sols_p, joinpath(seed_folder, "solution_evolution_plot.png"))

            return best_solution, logbook
        end

function test_vns!(instance::PVRPInstanceStruct, instance_name::String, num_runs::Int, save_folder::String, num_iterations::Int, acceptance_probability::Float64, acceptance_iterations::Int, no_improvement_iterations::Int)::Vector{Tuple{PVRPSolution, VNSLogbook, Bool, Int}}
    results = []

    for run in 1:num_runs

        # Generate a seed for the run
        seed = rand(1:10000)

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
