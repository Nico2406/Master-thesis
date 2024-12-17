using Revise
using VNS_PVRP
using VNS_PVRP.PVRPInstance: initialize_instance, plot_instance
using VNS_PVRP.Solution: display_solution, plot_solution, save_solution_to_yaml, load_solution_from_yaml, save_run_info_to_yaml
using VNS_PVRP.VNS: test_vns!, calculate_cost
using Random 
using Plots
using FilePathsBase: mkpath, joinpath
using YAML

function main()

    # Initialize instance
    instance_name = "p03"
    instance = initialize_instance("instances/$instance_name.txt")

    # Plot the instance
    plot = plot_instance(instance)
    display(plot)

    # Create a folder to save the solutions and run information
    save_folder = "/Users/nicoehler/Desktop/Masterarbeit Code/VNS_PVRP/Solutions"
    instance_folder = joinpath(save_folder, instance_name)
    mkpath(instance_folder)

    # Perform VNS test runs

    println("Performing VNS test runs on instance: $instance_name.txt")
    println("========================================")
    number_of_runs = 5
    results = test_vns!(instance, number_of_runs, instance_folder) 

    println("Instance: $instance_name.txt")
    println("Number of Vehicles: "    , instance.numberofvehicles)
    println("Vehicle Capacity: "      , instance.vehicleload) 
    println("Number of Nodes: "       , instance.numberofcustomers)   
    println("Number of Days: "        , instance.numberofdays)
    println("Number of Runs: "        , number_of_runs)
    println("========================================")

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
