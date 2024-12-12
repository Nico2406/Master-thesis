using Revise
using VNS_PVRP
using VNS_PVRP.PVRPInstance: initialize_instance, plot_instance
using VNS_PVRP.Solution: display_solution, plot_solution
using VNS_PVRP.VNS: test_vns!, calculate_cost
using Random  # Ensure this line is present
using Plots

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
