module VNS

using ..PVRPInstance: PVRPInstanceStruct
using ..Solution: PVRPSolution, VRPSolution, Route, recalculate_route!, remove_segment!, insert_segment!, validate_solution, display_solution, plot_solution, save_solution_to_yaml, save_run_info_to_yaml
using ..ConstructionHeuristics: nearest_neighbor
using ..LocalSearch: local_search!
using ..Shaking: shaking!
using Random  # Ensure this line is present
using Plots
using FilePathsBase: mkpath, joinpath
using YAML

export vns!, test_vns!

function vns!(instance::PVRPInstanceStruct, seed::Int, save_folder::String)::PVRPSolution
    Random.seed!(seed)
    # println("Random seed set to $seed")

    # Generate initial solution
    construction_method = "nearest_neighbor"
    current_solution = nearest_neighbor(instance)
    best_solution = deepcopy(current_solution)
    for day in keys(best_solution.tourplan)
        for route in best_solution.tourplan[day].routes
            recalculate_route!(route, instance)
        end
    end
    best_solution.plan_length = sum(route.length for day in keys(best_solution.tourplan) for route in best_solution.tourplan[day].routes)
    best_solution.plan_duration = sum(route.duration for day in keys(best_solution.tourplan) for route in best_solution.tourplan[day].routes)
    best_cost = best_solution.plan_length

    # Validate initial solution
    if !validate_solution(current_solution, instance)
        error("Initial solution is invalid.")
    end

    #display_solution(current_solution, instance, "Initial solution")
    #println("Initial solution cost: $best_cost")
    #println("Construction method: $construction_method")

    # Local search
    local_search_method = "2opt-first"
    for day in sort(collect(keys(current_solution.tourplan)))
        if isempty(current_solution.tourplan[day].routes)
            continue
        end
        for route in current_solution.tourplan[day].routes
            route.changed = true
        end
        Δ = local_search!(current_solution.tourplan[day], instance, local_search_method, 1000)
        # println("Total improvement for day $day: $Δ")
        if !validate_solution(current_solution, instance)
            error("Solution became invalid after local search on day $day.")
        end
    end

    for day in keys(current_solution.tourplan)
        for route in current_solution.tourplan[day].routes
            recalculate_route!(route, instance)
        end
    end
    current_solution.plan_length = sum(route.length for day in keys(current_solution.tourplan) for route in current_solution.tourplan[day].routes)
    current_solution.plan_duration = sum(route.duration for day in keys(current_solution.tourplan) for route in current_solution.tourplan[day].routes)
    current_cost = current_solution.plan_length

    #println("Cost after initial local search: $current_cost, Δ: $(current_cost - best_cost)")
    #println("Local search method: $local_search_method")

    # ganz nach oben
    no_improvement_counter = 0
    max_iterations = 100
    max_no_improvement = 10

    for iteration in 1:max_iterations
        #println("Iteration: $iteration")  # Debug print
        if no_improvement_counter >= max_no_improvement
            break
        end

        # Shaking step
        shaken_solution = deepcopy(current_solution)
        original_node_count = sum(length(route.visited_nodes) for day in keys(shaken_solution.tourplan) for route in shaken_solution.tourplan[day].routes)
        for day in sort(collect(keys(shaken_solution.tourplan)))
            if isempty(shaken_solution.tourplan[day].routes)
                continue
            end
            if length(shaken_solution.tourplan[day].routes) < 2
                continue
            end
            valid_day = true
            for route in shaken_solution.tourplan[day].routes
                if length(route.visited_nodes) <= 2
                    valid_day = false
                    break
                end
            end
            if !valid_day
                continue
            end
            try
                delta_shaking = shaking!(shaken_solution, instance, day)
                #println("Delta after shaking on day $day: $delta_shaking")
                if delta_shaking == Inf
                    continue
                end
                if !validate_solution(shaken_solution, instance)
                    println("Shaken solution is invalid after shaking on day $day. Skipping to next iteration.")
                    continue
                end
            catch e
                println("Shaking failed on day $day: ", e)
                continue
            end
        end

        # Ensure no routes are lost
        for day in sort(collect(keys(shaken_solution.tourplan)))
            shaken_solution.tourplan[day].routes = filter(route -> !(isempty(route.visited_nodes) || route.visited_nodes == [0, 0]), shaken_solution.tourplan[day].routes)
        end

        # Ensure no nodes are lost
        new_node_count = sum(length(route.visited_nodes) for day in keys(shaken_solution.tourplan) for route in shaken_solution.tourplan[day].routes)
        if original_node_count != new_node_count
            error("Nodes were lost during the shaking process.")
        end

        # Local search for changed routes
        for day in sort(collect(keys(shaken_solution.tourplan)))
            for route in shaken_solution.tourplan[day].routes
                if route.changed
                    #println("Performing local search on day $day, route")  # Debug print
                    Δ = local_search!(route, instance, local_search_method, 1000)
                    if !validate_solution(shaken_solution, instance)
                        println("Solution became invalid after local search on day $day. Skipping route.")
                        continue
                    end
                end
            end
        end

        for day in keys(shaken_solution.tourplan)
            for route in shaken_solution.tourplan[day].routes
                recalculate_route!(route, instance)
            end
        end
        shaken_solution.plan_length = sum(route.length for day in keys(shaken_solution.tourplan) for route in shaken_solution.tourplan[day].routes)
        shaken_solution.plan_duration = sum(route.duration for day in keys(shaken_solution.tourplan) for route in shaken_solution.tourplan[day].routes)
        shaken_cost = shaken_solution.plan_length

        #println("Cost after shaking and local search: $shaken_cost, Δ: $(shaken_cost - current_cost)")

        # Acceptance criterion
        if shaken_cost < best_cost
            best_solution = deepcopy(shaken_solution)
            best_cost = shaken_cost
            no_improvement_counter = 0
        else
            no_improvement_counter += 1
        end

        current_solution = deepcopy(shaken_solution)
        current_cost = shaken_cost
    end

    # Create a folder for each solution
    solution_folder = joinpath(save_folder, "solution_seed_$seed")
    mkpath(solution_folder)

    # Save the solution to a YAML file
    solution_filepath = joinpath(solution_folder, "solution.yaml")
    save_solution_to_yaml(best_solution, solution_filepath)

    # Save run information to a YAML file
    runtime = 0.0  # Placeholder for runtime, replace with actual runtime if available
    cost = best_solution.plan_length
    run_info_filepath = joinpath(solution_folder, "run_info.yaml")
    save_run_info_to_yaml(seed, runtime, cost, true, run_info_filepath)

    return best_solution
end

function test_vns!(instance::PVRPInstanceStruct, num_runs::Int, save_folder::String)
    results = []
    for i in 1:num_runs
        # Reinitialize instance to ensure a fresh start for each run
        fresh_instance = deepcopy(instance)
        seed = rand(1:10000)
        solution = vns!(fresh_instance, seed, save_folder)
        is_solution_valid = validate_solution(solution, fresh_instance)
        push!(results, (seed, solution, is_solution_valid))
        # println("Best solution cost: $(solution.plan_length)")
    end
    return results
end

end # module
