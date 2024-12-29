using Revise
using VNS_PVRP
using VNS_PVRP.PVRPInstance: initialize_instance, plot_instance
using VNS_PVRP.Solution: display_solution, plot_solution, save_solution_to_yaml, load_solution_from_yaml, save_run_info_to_yaml
using VNS_PVRP.VNS: test_vns!
using Random 
using Plots
using FilePathsBase: mkpath, joinpath
using YAML

function main()

    # Initialize instance
    instance_name = "p02"
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
    number_of_runs = 50
    results = test_vns!(instance, number_of_runs, instance_folder) 

    println("Instance: $instance_name.txt")
    println("Number of Vehicles: "    , instance.numberofvehicles)
    println("Vehicle Capacity: "      , instance.vehicleload) 
    println("Number of Nodes: "       , instance.numberofcustomers)   
    println("Number of Days: "        , instance.numberofdays)
    println("Number of Runs: "        , number_of_runs)
    println("========================================")

    best_solution = nothing
    best_length = Inf
    best_seed = nothing
    best_validity = false

    for (seed, solution, is_solution_valid) in results
        if solution.plan_length < best_length
            best_solution = solution
            best_length = solution.plan_length
            best_seed = seed
            best_validity = is_solution_valid
        end
    end

    if best_solution !== nothing
        println("========================================")
        println("Best Solution:")
        println("Seed: $best_seed")
        println("Solution valid: ", best_validity)
        println("Overall length: ", best_solution.plan_length)
        println("Overall duration: ", best_solution.plan_duration)
        plot = plot_solution(best_solution, instance)
        display(plot)
        display_solution(best_solution, instance, "Best Solution")
        println("========================================")
    else
        println("No valid solution found.")
    end
end

main()
