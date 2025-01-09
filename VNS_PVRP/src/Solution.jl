module Solution

# ...existing code...

# Plot the logbook data
function plot_logbook(logbook::VNSLogbook, instance_name::String, seed::Int, output_dir::String)::Plots.Plot
    # Plot best, best feasible, and current solution lengths
    sols_p = plot(
        logbook.iteration,
        [logbook.best_solution_length, logbook.best_feasible_solution_length, logbook.current_solution_length],
        label = ["Best Solution" "Best Feasible Solution" "Current Solution"],
        xlabel = "Iteration",
        ylabel = "Solution Length",
        title = "Solution Evolution",
        size = (1200, 800)
    )
    savefig(sols_p, joinpath(output_dir, instance_name, string(seed), "solution_evolution_plot.png"))

    # Plot parameters
    if !isempty(logbook.parameters)
        for (param_name, values) in logbook.parameters
            params_p = plot(
                logbook.iteration,
                values,
                label = [param_name],
                xlabel = "Iteration",
                ylabel = "Parameter Value",
                title = "$(param_name) Evolution",
                size = (1200, 800)
            )
            savefig(params_p, joinpath(output_dir, instance_name, string(seed), "$(param_name)_evolution_plot.png"))
        end
    end

    return sols_p
end

# ...existing code...

end # module
