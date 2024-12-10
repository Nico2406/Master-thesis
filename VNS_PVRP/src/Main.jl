include("PVRPInstance.jl")
include("PVRPSolution.jl")
include("ConstructionHeuristics.jl")
include("LocalSearch.jl")
include("Shaking.jl")
include("VNS.jl")

using Main.Instance: read_instance, PVRPInstance, plot_instance, initialize_instance
using Main.Solution: PVRPSolution, VRPSolution, plot_solution, plot_solution!, validate_solution, display_solution
using Main.VNS: test_vns!, calculate_cost
using Main.ConstructionHeuristics: nearest_neighbor, create_big_route, split_routes
using Main.LocalSearch: local_search!
using Main.Shaking: shaking!
using Random

function main()
    # Initialize instance
    instance = initialize_instance("instances/p03.txt")

    # Plot the instance
    plot = plot_instance(instance)
    display(plot)

    # Perform VNS test runs
    println("Performing VNS test runs...")
    results = test_vns!(instance, 5) 

    for (seed, solution, is_solution_valid) in results
        println("========================================")
        println("Seed: $seed")
        println("Vehicle Capacity: ", instance.vehicleload)
        display_solution(solution, instance, "Solution for seed $seed")
        plot = plot_solution(solution, instance)
        display(plot)
        println("Validating solution for seed $seed:")
        println("Solution valid: ", is_solution_valid)
        println("Overall length: ", calculate_cost(solution)) 
        println("Overall duration: ", solution.plan_duration)
        println("========================================")
    end
end

main()
