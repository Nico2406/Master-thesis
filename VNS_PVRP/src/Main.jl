using Revise
using VNS_PVRP
using VNS_PVRP.PVRPInstance: initialize_instance, plot_instance
using VNS_PVRP.Solution: display_solution, plot_logbook, plot_solution
using VNS_PVRP.VNS: vns!
using FilePathsBase: mkpath

function main()
    instance_name = "p02"
    instance = initialize_instance("instances/$instance_name.txt")
    plot = plot_instance(instance)
    display(plot)

    save_folder = "/Users/nicoehler/Desktop/Masterarbeit Code/VNS_PVRP/Solutions"
    mkpath(save_folder)

    println("Running VNS...")
    best_solution, logbook, seed = vns!(instance, instance_name, save_folder)

    println("Plotting Logbook...")
    logbook_plot = plot_logbook(logbook, instance_name, "test_run", save_folder)
    display(logbook_plot)

    println("VNS completed. Best solution length: ", best_solution.plan_length)
    println("Seed used: ", seed)
    display_solution(best_solution, instance, "Final Solution")
    solution_plot = plot_solution(best_solution, instance)
    display(solution_plot)
end

main()
