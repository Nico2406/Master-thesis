using Revise
using VNS_PVRP
using VNS_PVRP.PVRPInstance: initialize_instance, plot_instance
using VNS_PVRP.Solution: display_solution, plot_logbook, plot_solution, validate_solution, calculate_kpis_with_treatment
using VNS_PVRP.VNS: test_vns!
using FilePathsBase: mkpath

function main()
    # Parameter für KPI-Berechnung
    region = :urban
    bring_participation = 0.8  # Anteil, der sich am Bringsystem beteiligt
    ev_share = 0.3  # Anteil Elektrofahrzeuge
    average_idle_time_per_stop = 0.1  # Durchschnittliche Leerlaufzeit pro Stopp in Stunden

    instance_name = "p05"
    instance = initialize_instance("instances/$instance_name.txt")
    plot = plot_instance(instance)
    display(plot)

    save_folder = "/Users/nicoehler/Desktop/Masterarbeit Code/VNS_PVRP/Solutions"
    mkpath(save_folder)

    num_runs = 1
    println("Running VNS for $num_runs runs...")
    results = test_vns!(instance, instance_name, num_runs, save_folder)

    # Find the best solution from the results
    best_result_index = argmin([result[2].plan_length for result in results])
    best_result = results[best_result_index]
    best_solution, logbook, seed, is_solution_valid = best_result[2], best_result[3], best_result[1], best_result[4]

    println("Plotting Logbook for the best run...")
    logbook_plot = plot_logbook(logbook, instance_name, "best_run", save_folder)
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
    println("====================================")

    display_solution(best_solution, instance, "Final Solution")

    # Validate the final solution
    println("Final solution validation: ", is_solution_valid ? "Valid" : "Invalid")

    # Berechnung der KPIs
    println("\nCalculating KPIs with the following parameters:")
    println("Region: ", region)
    println("Bring Participation: ", bring_participation)
    println("EV Share: ", ev_share)
    println("Average Idle Time per Stop (h): ", average_idle_time_per_stop)
    kpis = calculate_kpis_with_treatment(
        best_solution, instance, region=region,
        bring_participation=bring_participation,
        ev_share=ev_share,
        average_idle_time_per_stop=average_idle_time_per_stop
    )
    println("\nKPIs Summary:")
    for (key, value) in kpis
        println(key, ": ", value)
    end

    # Print the portion of the total energy caused by each phase
    total_energy = kpis["Total Energy (MJ)"]
    transport_energy = kpis["Transport Phase (Ehaul)"]["Energy (MJ)"]
    collection_energy = kpis["Collection Phase (Edrivecollect)"]["Energy (MJ)"]
    stop_energy = kpis["Stop Phase (Estopcollect)"]["Energy (MJ)"]
    idle_energy = kpis["Idle Phase"]["Energy (MJ)"]

    println("\nEnergy Portion by Phase:")
    println("Transport Phase: ", transport_energy / total_energy * 100, "%")
    println("Collection Phase: ", collection_energy / total_energy * 100, "%")
    println("Stop Phase: ", stop_energy / total_energy * 100, "%")
    println("Idle Phase: ", idle_energy / total_energy * 100, "%")

    # Plot and display the best solution
    solution_plot = plot_solution(best_solution, instance)
    display(solution_plot)
end

main()