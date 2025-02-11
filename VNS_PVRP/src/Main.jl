using Revise
using VNS_PVRP
using VNS_PVRP.PVRPInstance: initialize_instance, plot_instance, read_distance_matrix
using VNS_PVRP.Solution: display_solution, plot_logbook, plot_solution, validate_solution, calculate_kpis_with_treatment, display_kpis, load_solution_and_calculate_kpis, Calculate_KPIs_real_instance, display_kpis_real_instance, load_solution_and_calculate_KPIs_real_instance
using VNS_PVRP.VNS: test_vns!, optimize_loaded_solution!
using FilePathsBase: mkpath
using Dates: now, Minute, Dates

function main()
    # Parameters for KPI calculation
    region = :urban  # Options: :urban, :suburban, :rural (default: :urban)
    bring_participation = 1.0  # Proportion participating in the bring system (default: 1.0)
    ev_share = 0.0  # Proportion of electric vehicles (ensure this is a float) (default: 0.0)
    average_idle_time_per_stop = 0.01  # Average idle time per stop in hours (default: 0.01)
    compacting_energy = 10.0  # Energy needed for compacting the waste (MJ) (default: 10.0)
    stops_per_compacting = 5  # Number of stops needed for one time compacting (default: 5)
    average_speed = 40.0  # Average speed (km/h) (default: 40.0)
    treatment_distance = 10.0  # Distance to treatment plant (km) (default: 10.0)
    average_load = 5.0  # Average load (tons) (default: 5.0)
    stop_energy = 2.3  # Energy consumption per stop (MJ) (default: 2.3)
    energy_per_km = 9.0  # Energy consumption per km (MJ) (default: 9.0)
    idle_energy = 36.0  # Idle energy consumption (MJ/h) (default: 36.0)
    waste_amount = 5.0  # Amount of waste (tons) (default: 5.0)

    # VNS algorithm parameters
    num_iterations = 1000 # Number of iterations for the VNS algorithm
    acceptance_probability = 0.00  # Acceptance probability for the VNS algorithm
    acceptance_iterations = 50  # Number of acceptance iterations for the VNS algorithm
    no_improvement_iterations = 1000 # Number of iterations without improvement before stopping the VNS algorithm

    # Instance configuration
    instance_base_name = "pr05"  # Base name of the instance
    system_type = ""  # Options: "Holsystem", "Holsystem1min", "Bringsystem"
    instance_name = instance_base_name * "_" * system_type  # Full instance name
    use_cordeau_instance = true  # Set to true if using Cordeau instances

    if use_cordeau_instance
        instance_file_path = "instances/" * instance_base_name * ".txt"
        distance_matrix_filepath = nothing
    else
        instance_file_path = "real_instances/" * instance_name * "_ohneIFs.yaml"
        distance_matrix_filepath = "real_instances/mtx_" * instance_base_name * "_2089ohneIF_min.txt"
    end

    # Initialize instance
    if isnothing(distance_matrix_filepath)
        instance = VNS_PVRP.PVRPInstance.initialize_instance(instance_file_path)
    else
        instance = VNS_PVRP.PVRPInstance.initialize_instance(instance_file_path, distance_matrix_filepath)
    end

    # Plot the instance
    plot = VNS_PVRP.PVRPInstance.plot_instance(instance)
    display(plot)

    # Save folder for the solutions
    save_folder = "/Users/nicoehler/Desktop/Masterarbeit Code/VNS_PVRP/Solutions/" * instance_name
    mkpath(save_folder)

    # Run the VNS algorithm with multiple runs
    num_runs = 1
    
    println("Running VNS for $num_runs runs...")
    start_time = now()
    results = test_vns!(instance, instance_name, num_runs, save_folder, num_iterations, acceptance_probability, acceptance_iterations, no_improvement_iterations)
    end_time = now()
    elapsed_time = end_time - start_time

    # Find the best solution from the results
    best_result_index = argmin([result[1].plan_length for result in results])
    best_result = results[best_result_index]
    best_solution, logbook, is_solution_valid, seed = best_result[1], best_result[2], best_result[3], best_result[4]

    # Display the logbook
    logbook_plot = plot_logbook(logbook, instance_name, seed, save_folder)
    display(logbook_plot)

    println("VNS completed.")
    println("====================================")
    println("Instance Name: ", instance_base_name)
    println("Number of Vehicles: ", instance.numberofvehicles)
    println("Number of Days: ", instance.numberofdays)
    println("Number of Customers: ", instance.numberofcustomers)
    println("Vehicle Capacity: ", instance.vehicleload)
    println("Maximum Route Duration: ", instance.maximumrouteduration)
    println("====================================")
    println("Best Solution Length: ", best_solution.plan_length)
    println("Total Plan Duration: ", best_solution.plan_duration)
    println("====================================")
    println("Seed used: ", seed)
    println("Number of Runs: ", num_runs)
    println("Number of Iterations in the VNS: ", num_iterations)
    elapsed_time_minutes = div(Dates.value(elapsed_time), 60000)
    elapsed_time_seconds = round(Dates.value(elapsed_time) / 1000 % 60, digits=2)
    println("Elapsed Time: ", elapsed_time)
    println("Elapsed Time (minutes): ", elapsed_time_minutes, " minutes and ", elapsed_time_seconds, " seconds")
    println("====================================")

    println("Best Solution found in run $best_result_index")
    println("====================================")

    # Display the best solution
    # display_solution(best_solution, instance, "Final Solution")

    # Validate the final solution
    println("Final solution validation: ", is_solution_valid ? "Valid" : "Invalid")
    println("====================================")

    # Display KPIs
    # display_kpis(best_solution, instance, region, bring_participation, ev_share, average_idle_time_per_stop, compacting_energy, stops_per_compacting, average_speed, treatment_distance, average_load, stop_energy, energy_per_km, idle_energy)

    # Plot and display the best solution
    solution_plot = plot_solution(best_solution, instance)
    display(solution_plot)

    display_solution(best_solution, instance, "Final Solution")

    
    # Load a solution from YAML and calculate KPIs
    solution_filepath = "/Users/nicoehler/Desktop/Masterarbeit Code/VNS_PVRP/Solutions/Weiz_BIO/3491/final_solution.yaml"
    println("====================================")
    #println("Loading solution from solution_filepath and calculating KPIs...")
    #load_solution_and_calculate_kpis(solution_filepath, instance, region, bring_participation, ev_share, average_idle_time_per_stop, compacting_energy, stops_per_compacting, average_speed, treatment_distance, average_load, stop_energy, energy_per_km, idle_energy)

    

    """
    println("Optimizing a loaded solution...")
    optimized_results = optimize_loaded_solution!(solution_filepath, instance, instance_name, num_runs, save_folder, num_iterations, acceptance_probability, acceptance_iterations, no_improvement_iterations)
    optimized_solution = optimized_results[1][1]  # Extract the PVRPSolution object
    display_solution(optimized_solution, instance, "Optimized Solution")
    """
   
    """
    # Calculate KPIs of the real Instance
    distance_matrix_Distance = read_distance_matrix("real_instances/mtx_" * instance_name * "_2089ohneIF_m.txt")
    load_solution_and_calculate_KPIs_real_instance(solution_filepath, instance, region, bring_participation, ev_share, compacting_energy, stops_per_compacting, treatment_distance, average_load, waste_amount, stop_energy, energy_per_km, distance_matrix_Distance, instance.distance_matrix)


    
    println("Calculating KPIs for the real instance...")
    display_kpis_real_instance(
        best_solution[1], 
        instance, 
        region, 
        bring_participation, 
        ev_share, 
        compacting_energy, 
        stops_per_compacting, 
        treatment_distance, 
        average_load, 
        stop_energy, 
        energy_per_km, 
        distance_matrix_Distance, 
        instance.distance_matrix
    )
    """

    end

main()