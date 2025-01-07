using Revise
using VNS_PVRP
using VNS_PVRP.PVRPInstance: initialize_instance, plot_instance
using VNS_PVRP.Solution: display_solution, plot_logbook, plot_solution, validate_solution
using VNS_PVRP.VNS: vns!, test_vns!
using FilePathsBase: mkpath

function main()
    instance_name = "p02"
    instance = initialize_instance("instances/$instance_name.txt")
    plot = plot_instance(instance)
    display(plot)

    save_folder = "/Users/nicoehler/Desktop/Masterarbeit Code/VNS_PVRP/Solutions"
    mkpath(save_folder)

    num_runs = 10
    println("Running VNS for $num_runs runs...")
    results = test_vns!(instance, instance_name, num_runs, save_folder)

    println("Plotting Logbook for the last run...")
    best_solution, logbook, seed, is_solution_valid = results[end][2], results[end][3], results[end][1], results[end][4]
    logbook_plot = plot_logbook(logbook, instance_name, "test_run", save_folder)
    display(logbook_plot)

    println("VNS completed.")
    println("====================================")
    println("Instance Name: ", instance_name)
    println("Number of Vehicles: ", instance.numberofvehicles)
    println("Number of Days: ", instance.numberofdays)
    println("Number of Nodes: ", instance.numberofcustomers + 1)  # Including depot
    println("Vehicle Capacity: ", instance.vehicleload)
    println("Maximum Route Duration: ", instance.maximumrouteduration)
    println("Best Solution Length: ", best_solution.plan_length)
    println("Total Plan Duration: ", best_solution.plan_duration)
    println("Seed used: ", seed)
    println("Number of Runs: ", num_runs)
    println("====================================")

    display_solution(best_solution, instance, "Final Solution")
    solution_plot = plot_solution(best_solution, instance)
    display(solution_plot)

    # Validate the final solution
    println("Final solution validation: ", is_solution_valid ? "Valid" : "Invalid")
end

main()
