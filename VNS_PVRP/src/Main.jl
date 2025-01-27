using Revise
using VNS_PVRP
using VNS_PVRP.PVRPInstance: initialize_instance, plot_instance
using VNS_PVRP.Solution: display_solution, plot_logbook, plot_solution, validate_solution, calculate_kpis_with_treatment, display_kpis, load_solution_and_calculate_kpis
using VNS_PVRP.VNS: test_vns!
using FilePathsBase: mkpath
using Dates: now

function main()
    # Parameters for KPI calculation
    region = :urban  # Options: :urban, :suburban, :rural
    bring_participation = 0.8  # Proportion participating in the bring system
    ev_share = 0.0  # Proportion of electric vehicles (ensure this is a float)
    average_idle_time_per_stop = 0.1  # Average idle time per stop in hours
    compacting_energy = 10.0  # Energy needed for compacting the waste (MJ)
    stops_per_compacting = 5  # Number of stops needed for one time compacting
    average_speed = 40.0  # Average speed (km/h)
    treatment_distance = 10.0  # Distance to treatment plant (km)
    average_load = 5.0  # Average load (tons)
    stop_energy = 2.3  # Energy consumption per stop (MJ)
    energy_per_km = 9.0  # Energy consumption per km (MJ)
    idle_energy = 36.0  # Idle energy consumption (MJ/h)
    num_iterations = 10000  # Number of iterations for the VNS algorithm

# Instanzname und Dateipfade
instance_name = "Weiz_BIO_alt"
instance_file_path = "real_instances/" * instance_name * ".yaml"
distance_matrix_filepath = "real_instances/" * instance_name * "_mtx.txt"

# Initialize instance
instance = initialize_instance(instance_file_path, distance_matrix_filepath)
    plot = plot_instance(instance)
    display(plot)

    save_folder = "/Users/nicoehler/Desktop/Masterarbeit Code/VNS_PVRP/Solutions"
    mkpath(save_folder)

    num_runs = 1
    println("Running VNS for $num_runs runs...")
    start_time = now()
    results = test_vns!(instance, instance_name, num_runs, save_folder, num_iterations)
    end_time = now()
    elapsed_time = end_time - start_time

    # Find the best solution from the results
    best_result_index = argmin([result[1].plan_length for result in results])
    best_result = results[best_result_index]
    best_solution, logbook, is_solution_valid, seed = best_result[1], best_result[2], best_result[3], best_result[4]

    logbook_plot = plot_logbook(logbook, instance_name, seed, save_folder)
    display(logbook_plot)

    println("VNS completed.")
    println("====================================")
    println("Instance Name: ", instance_name)
    println("Number of Vehicles: ", instance.numberofvehicles)
    println("Number of Days: ", instance.numberofdays)
    println("Number of Nodes: ", instance.numberofcustomers)
    println("Vehicle Capacity: ", instance.vehicleload)
    println("Maximum Route Duration: ", instance.maximumrouteduration)
    println("Best Solution Length: ", best_solution.plan_length)
    println("Total Plan Duration: ", best_solution.plan_duration)
    println("Seed used: ", seed)
    println("Number of Runs: ", num_runs)
    println("Number of Iterations in the VNS: ", num_iterations)
    println("Elapsed Time: ", elapsed_time)
    println("====================================")

    display_solution(best_solution, instance, "Final Solution")

    # Validate the final solution
    println("Final solution validation: ", is_solution_valid ? "Valid" : "Invalid")
    println("====================================")

    # Display KPIs
    display_kpis(best_solution, instance, region, bring_participation, ev_share, average_idle_time_per_stop, compacting_energy, stops_per_compacting, average_speed, treatment_distance, average_load, stop_energy, energy_per_km, idle_energy)

    # Plot and display the best solution
    solution_plot = plot_solution(best_solution, instance)
    display(solution_plot)

    # Load a solution from YAML and calculate KPIs
    solution_filepath = "/Users/nicoehler/Desktop/Masterarbeit Code/VNS_PVRP/Solutions/p05/3319/solution.yaml"
    println("====================================")
    println("Loading solution from $solution_filepath and calculating KPIs...")
    #load_solution_and_calculate_kpis(solution_filepath, instance, region, bring_participation, ev_share, average_idle_time_per_stop, compacting_energy, stops_per_compacting, average_speed, treatment_distance, average_load, stop_energy, energy_per_km, idle_energy)
end

main()