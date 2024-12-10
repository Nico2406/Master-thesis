module VNS

using Main.Instance: PVRPInstance
using Main.Solution: PVRPSolution, VRPSolution, Route, recalculate_route!, remove_segment!, insert_segment!, validate_solution, display_solution, plot_solution
using Main.ConstructionHeuristics: nearest_neighbor
using Main.LocalSearch: local_search!
using Main.Shaking: shaking!
using Random

export vns!, test_vns!, calculate_cost

function vns!(instance::PVRPInstance, seed::Int)::PVRPSolution
    Random.seed!(seed)
    # println("Random seed set to $seed")

    # Generate initial solution
    construction_method = "nearest_neighbor"
    current_solution = nearest_neighbor(instance)
    best_solution = deepcopy(current_solution)
    best_cost = calculate_cost(best_solution)

    # Validate initial solution
    if !validate_solution(current_solution, instance)
        error("Initial solution is invalid.")
    end

    display_solution(current_solution, instance, "Initial solution")
    println("Initial solution cost: $best_cost")
    println("Construction method: $construction_method")

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

    current_cost = calculate_cost(current_solution)
    println("Cost after initial local search: $current_cost, Δ: $(current_cost - best_cost)")
    println("Local search method: $local_search_method")

    # ganz nach oben
    no_improvement_counter = 0
    max_iterations = 100
    max_no_improvement = 10

    for iteration in 1:max_iterations
        println("Iteration: $iteration")  # Debug print
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
                println("Delta after shaking on day $day: $delta_shaking")
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
                    println("Performing local search on day $day, route")  # Debug print
                    Δ = local_search!(route, instance, local_search_method, 1000)
                    if !validate_solution(shaken_solution, instance)
                        println("Solution became invalid after local search on day $day. Skipping route.")
                        continue
                    end
                end
            end
        end

        shaken_cost = calculate_cost(shaken_solution)
        println("Cost after shaking and local search: $shaken_cost, Δ: $(shaken_cost - current_cost)")

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

    return best_solution
end

function test_vns!(instance::PVRPInstance, num_runs::Int)
    results = []
    for i in 1:num_runs
        # Reinitialize instance to ensure a fresh start for each run
        fresh_instance = deepcopy(instance)
        seed = rand(1:10000)
        solution = vns!(fresh_instance, seed)
        is_solution_valid = validate_solution(solution, fresh_instance)
        push!(results, (seed, solution, is_solution_valid))
        # println("Best solution cost: $(calculate_cost(solution))")  # Remove this line
    end
    return results
end

function calculate_cost(solution::PVRPSolution)::Float64
    total_cost = 0.0
    for day in sort(collect(keys(solution.tourplan)))
        for route in solution.tourplan[day].routes
            total_cost += route.length
        end
    end
    return total_cost
end

end # module
