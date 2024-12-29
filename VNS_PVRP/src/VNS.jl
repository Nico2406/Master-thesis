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

    # Generate initial solution
    current_solution = nearest_neighbor(instance)
    best_solution = deepcopy(current_solution)
    for day in keys(best_solution.tourplan)
        for route in best_solution.tourplan[day].routes
            recalculate_route!(route, instance)
        end
    end

    if !validate_solution(current_solution, instance)
        error("Initial solution is invalid.")
    end

    max_iterations = 100
    for iteration in 1:max_iterations
        try
            for day in keys(current_solution.tourplan)
                shaking!(current_solution, instance, day)
            end

            # Perform local search on all changed routes
            for day in keys(current_solution.tourplan)
                for route in current_solution.tourplan[day].routes
                    if route.changed
                        local_search!(route, instance, "2opt-first", 1000)
                        route.changed = false  # Reset the changed flag after local search
                    end
                end
            end

            if validate_solution(current_solution, instance)
                best_solution = deepcopy(current_solution)
            end
        catch e
            println("Error during iteration $iteration: $e")
        end
    end

    save_solution_to_yaml(best_solution, joinpath(save_folder, "solution.yaml"))
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
