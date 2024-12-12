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
    instance = initialize_instance("instances/p03.txt")

    # Plot the instance
    plot = plot_instance(instance)
    display(plot)

    # Create a folder to save the solutions and run information
    save_folder = "/Users/nicoehler/Desktop/Masterarbeit Code/VNS_PVRP/Solutions"
    mkpath(save_folder)

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

        # Create a folder for each solution
        solution_folder = joinpath(save_folder, "solution_seed_$seed")
        mkpath(solution_folder)

        # Save the solution to a YAML file
        solution_filepath = joinpath(solution_folder, "solution.yaml")
        save_solution_to_yaml(solution, solution_filepath)

        # Save run information to a YAML file
        runtime = 0.0  # Placeholder for runtime, replace with actual runtime if available
        cost = calculate_cost(solution)
        run_info_filepath = joinpath(solution_folder, "run_info.yaml")
        save_run_info_to_yaml(seed, runtime, cost, is_solution_valid, run_info_filepath)
    end


    # Example of loading a solution from a YAML file
    loaded_solution_filepath = joinpath(save_folder, "solution_seed_2019", "solution.yaml")
    loaded_solution = load_solution_from_yaml(loaded_solution_filepath)
    display_solution(loaded_solution, instance, "Loaded Solution for seed 2019")
end

main()
