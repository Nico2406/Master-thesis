module VNS

using ..PVRPInstance: PVRPInstanceStruct
using ..Solution: PVRPSolution, VRPSolution, Route, recalculate_route!, remove_segment!, insert_segment!, validate_solution, display_solution, plot_solution, save_solution_to_yaml, save_run_info_to_yaml, VNSLogbook, initialize_logbook, update_logbook!, save_logbook_to_yaml, recalculate_plan_length!
using ..ConstructionHeuristics: nearest_neighbor
using ..LocalSearch: local_search!
using ..Shaking: shaking!, change_visit_combinations!
using Random
using FilePathsBase: mkpath, joinpath

export vns!, test_vns!

function vns!(instance::PVRPInstanceStruct, instance_name::String, save_folder::String; seed::Int=rand(1:10000))::Tuple{PVRPSolution, VNSLogbook, Int}
    Random.seed!(seed)

    # Initialize the logbook
    logbook = initialize_logbook()

    # Generate initial solution
    current_solution = nearest_neighbor(instance)
    best_solution = deepcopy(current_solution)
    for day in keys(best_solution.tourplan)
        for route in best_solution.tourplan[day].routes
            recalculate_route!(route, instance)
        end
    end

    # Validate initial solution
    is_current_solution_feasible = validate_solution(current_solution, instance)
    if !is_current_solution_feasible
        error("Initial solution is invalid.")
    end

    max_iterations = 1000
    for iteration in 1:max_iterations
        try
            for day in keys(current_solution.tourplan)
                shaking!(current_solution, instance, day)
            end

            # Apply change visit combinations
            change_visit_combinations!(current_solution, instance)

            # Perform local search on all changed routes
            for day in keys(current_solution.tourplan)
                for route in current_solution.tourplan[day].routes
                    if route.changed
                        local_search!(route, instance, "2opt-first", 1000)
                        route.changed = false  # Reset the changed flag after local search
                    end
                end
            end

            # Recalculate the total plan length and duration
            recalculate_plan_length!(current_solution)

            # Validate the current solution
            is_current_solution_feasible = validate_solution(current_solution, instance)

            # Update the best solution if feasible
            if is_current_solution_feasible && current_solution.plan_length < best_solution.plan_length
                best_solution = deepcopy(current_solution)
            end

            # Update logbook
            update_logbook!(
                logbook,
                iteration,
                current_solution.plan_length,
                best_solution.plan_length,
                best_solution.plan_length,  # Placeholder for best feasible
                Dict("destroy_param" => 0.5),
                feasible=is_current_solution_feasible
            )
        catch e
            println("Error during iteration $iteration: $e")
        end
    end

    # Set the initial visit combination to the visits of the best solution
    for day in keys(best_solution.tourplan)
        for route in best_solution.tourplan[day].routes
            for node in route.visited_nodes
                if node != 0
                    instance.nodes[node + 1].initialvisitcombination[day] = true
                end
            end
        end
    end

    # Create the necessary directories
    instance_folder = joinpath(save_folder, instance_name)
    seed_folder = joinpath(instance_folder, string(seed))
    mkpath(seed_folder)

    # Save the solution, logbook, and run information
    save_solution_to_yaml(best_solution, joinpath(seed_folder, "solution.yaml"))
    save_logbook_to_yaml(logbook, joinpath(seed_folder, "logbook.yaml"))
    save_run_info_to_yaml(seed, 0.0, best_solution.plan_length, is_current_solution_feasible, joinpath(seed_folder, "run_info.yaml"))

    return best_solution, logbook, seed
end

function test_vns!(instance::PVRPInstanceStruct, instance_name::String, num_runs::Int, save_folder::String)
    results = []
    for i in 1:num_runs
        # Reinitialize instance to ensure a fresh start for each run
        fresh_instance = deepcopy(instance)
        best_solution, logbook, seed = vns!(fresh_instance, instance_name, save_folder)
        is_solution_valid = validate_solution(best_solution, fresh_instance)
        push!(results, (seed, best_solution, logbook, is_solution_valid))
    end
    return results
end

end # module
