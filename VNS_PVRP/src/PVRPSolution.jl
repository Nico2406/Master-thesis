module Solution

using ..PVRPInstance: PVRPInstanceStruct, Node, plot_instance, get_fitting_layout
using Plots
using YAML
using YAML: write_file, load_file

export Route, PVRPSolution, VRPSolution, plot_solution, plot_solution!, validate_route, validate_solution, recalculate_route!, remove_segment!, insert_segment, display_solution, save_solution_to_yaml, load_solution_from_yaml, save_run_info_to_yaml, save_logbook_to_yaml, plot_logbook, recalculate_plan_length!, calculate_kpis_with_treatment, display_kpis, run_parameter_study

# Define a mutable struct to represent a route
mutable struct Route
    visited_nodes::Vector{Int64}
    load::Float64
    length::Float64
    cost::Float64
    duration::Float64
    feasible::Bool
    changed::Bool
end

# Constructor for Route with default values for cost, duration, feasible, and changed
function Route(visited_nodes::Vector{Int}, load::Float64, length::Float64)
    Route(visited_nodes, load, length, 0.0, 0.0, true, false)
end

# Define a mutable struct to represent a VRP solution
mutable struct VRPSolution
    routes::Vector{Route}
    total_duration::Float64
    total_length::Float64
end

# Define a comparison method for VRPSolution based on total duration
Base.isless(a::VRPSolution, b::VRPSolution) = a.total_duration < b.total_duration

# Define a mutable struct to represent a PVRP solution
mutable struct PVRPSolution
    tourplan::Dict{Int, VRPSolution}
    plan_length::Float64
    plan_duration::Float64
end

# Constructor for PVRPSolution with default values
function PVRPSolution(instance::PVRPInstanceStruct)
    PVRPSolution(Dict{Int, VRPSolution}(), 0.0, 0.0)
end

# Function to plot the solution for all days
function plot_solution(sol::PVRPSolution, inst::PVRPInstanceStruct)
    l = get_fitting_layout(inst.numberofdays)
 
    plots = Dict{Int, Plots.Plot}()
    for (day, vrp) in sol.tourplan
        p = plot_instance(inst)
        plot_solution!(p, vrp, inst, day)  # Tag wird übergeben, um pro Tag die richtige Route anzuzeigen
        plots[day] = p
    end
 
    sorted_plots_by_days = sort(collect(plots), by = x -> x[1])
    p = plot([p for (d, p) in sorted_plots_by_days]..., layout = l)
    return p
end

# Function to plot the solution for a specific day
function plot_solution!(p::Plots.Plot, sol::VRPSolution, inst::PVRPInstanceStruct, day::Int)
    colours = [
        :firebrick3,
        :chartreuse4,
        :darkgoldenrod2,
        :cornflowerblue,
        :palevioletred,
        :cyan4,
        :orangered,
        :dodgerblue4,
        :mediumorchid4,
        :grey57,
        :peachpuff2,
        :darkgreen,
        :tomato,
        :saddlebrown,
        :lightgoldenrod,
    ]
    
    for (i, route) in enumerate(sol.routes)
        # Adjust indices so that `0` (Depot) corresponds to `nodes[1]`, and increment all others by 1
        x = [inst.nodes[1].x; [inst.nodes[n + 1].x for n in route.visited_nodes[2:end]]; inst.nodes[1].x]
        y = [inst.nodes[1].y; [inst.nodes[n + 1].y for n in route.visited_nodes[2:end]]; inst.nodes[1].y]
        
        plot!(p, x, y, linecolor = colours[mod(i - 1, length(colours)) + 1], title = "day $day", label = "")
    end
    
    return p
end

# Function to recalculate the properties of a route
function recalculate_route!(route::Route, instance::PVRPInstanceStruct)
    route.load = 0.0
    route.length = 0.0
    route.duration = 0.0
    for i in 1:(length(route.visited_nodes) - 1)
        from_node = route.visited_nodes[i]
        to_node = route.visited_nodes[i + 1]
        if from_node < 0 || from_node >= size(instance.distance_matrix, 1) || to_node < 0 || to_node >= size(instance.distance_matrix, 2)
            continue
        end
        route.length += instance.distance_matrix[from_node + 1, to_node + 1]
        route.duration += instance.distance_matrix[from_node + 1, to_node + 1]
        if to_node != 0  # Exclude depot
            # Adjust the index to access the correct node data
            node_index = to_node + 1
            route.load += instance.nodes[node_index].demand / instance.nodes[node_index].frequency
            route.duration += instance.nodes[node_index].service_time  # Ensure service duration is added
        end
    end
    # Ensure the route length and duration are correctly updated
    route.length += instance.distance_matrix[route.visited_nodes[end] + 1, 1]
    route.duration += instance.distance_matrix[route.visited_nodes[end] + 1, 1]
    
    # Mark the route as not feasible if the load exceeds the vehicle capacity or the duration exceeds the maximum route duration
    route.feasible = route.load <= instance.vehicleload && route.duration <= instance.maximumrouteduration
end

# Function to remove a segment from a route and recalculate its properties
function remove_segment!(route::Route, start_idx::Int, segment_length::Int, instance::PVRPInstanceStruct)::Float64
    if start_idx < 1 || start_idx + segment_length - 1 > length(route.visited_nodes)
        error("Invalid segment range: out of bounds.")
    end

    # Extract segment to be removed
    segment = route.visited_nodes[start_idx:(start_idx + segment_length - 1)]

    # Calculate delta before removal
    prev_node = start_idx > 1 ? route.visited_nodes[start_idx - 1] : 0
    next_node = start_idx + segment_length <= length(route.visited_nodes) ? route.visited_nodes[start_idx + segment_length] : 0
    delta = instance.distance_matrix[prev_node + 1, next_node + 1] - 
            instance.distance_matrix[prev_node + 1, segment[1] + 1] -
            instance.distance_matrix[segment[end] + 1, next_node + 1]

    # Remove segment from route
    deleteat!(route.visited_nodes, start_idx:(start_idx + segment_length - 1))

    # Recalculate route properties
    recalculate_route!(route, instance)

    return delta
end

# Function to insert a segment into a route and recalculate its properties
function insert_segment!(route::Route, start_idx::Int, segment::Vector{Int}, instance::PVRPInstanceStruct)::Float64
    if start_idx < 1 || start_idx > length(route.visited_nodes) + 1
        error("Invalid insertion index: out of bounds.")
    end

    # Calculate delta before insertion
    prev_node = start_idx > 1 ? route.visited_nodes[start_idx - 1] : 0
    next_node = start_idx <= length(route.visited_nodes) ? route.visited_nodes[start_idx] : 0
    delta = -instance.distance_matrix[prev_node + 1, next_node + 1] +
            instance.distance_matrix[prev_node + 1, segment[1] + 1] +
            instance.distance_matrix[segment[end] + 1, next_node + 1]

    # Insert segment into route
    for (i, node) in enumerate(segment)
        insert!(route.visited_nodes, start_idx + i - 1, node)
    end

    # Recalculate route properties
    recalculate_route!(route, instance)

    return delta
end

# Function to validate a route for a specific day
function validate_route(route::Route, instance::PVRPInstanceStruct, day::Int)::Bool
    recalculate_route!(route, instance)

    # Temporary: Print capacity issues instead of failing validation
    if route.load > instance.vehicleload
        #println("Warning: Load exceeds vehicle capacity on Day $day. Route Load: $(route.load), Vehicle Capacity: $(instance.vehicleload).")
    end

    if route.length + instance.distance_matrix[route.visited_nodes[end] + 1, 1] > instance.maximumrouteduration
        #println("Constraint violated: Route length exceeds maximum route duration.")
    end

    for node in route.visited_nodes[2:end-1]
        if node > 0 && !instance.nodes[node + 1].initialvisitcombination[day]
            println("Constraint violated: Node $node is not allowed to be visited on day $day.")
            return false
        end
    end

    calculated_load = sum(instance.nodes[node + 1].demand / instance.nodes[node + 1].frequency for node in route.visited_nodes[2:end-1] if node > 0)
    if calculated_load != route.load
        println("Constraint violated: Calculated load does not match route's load.")
        return false
    end

    calculated_length = sum(instance.distance_matrix[route.visited_nodes[i] + 1, route.visited_nodes[i + 1] + 1] for i in 1:(length(route.visited_nodes) - 1))
    if calculated_length != route.length
        println("Constraint violated: Calculated length does not match route's length.")
        return false
    end

    # Check if all the demand is picked up for each customer
    customer_demands = Dict{Int, Float64}()
    for node in route.visited_nodes[2:end-1]
        if node > 0
            customer_demands[node] = get(customer_demands, node, 0.0) + instance.nodes[node + 1].demand / instance.nodes[node + 1].frequency
        end
    end

    for (node_id, demand) in customer_demands
        if demand != instance.nodes[node_id + 1].demand / instance.nodes[node_id + 1].frequency
            println("Constraint violated: Demand for node $node_id is not fully picked up. Expected: $(instance.nodes[node_id + 1].demand / instance.nodes[node_id + 1].frequency), Found: $demand")
            return false
        end
    end

    return true
end

# Function to validate the entire solution
function validate_solution(sol::PVRPSolution, inst::PVRPInstanceStruct)::Bool
    valid = true
    for (day, vrp_solution) in sol.tourplan
        for route in vrp_solution.routes
            if !validate_route(route, inst, day)
                println("Warning: Route on Day $day failed validation.")
                valid = false
            end
        end
    end
    recalculate_plan_length!(sol)
    return valid
end


# Function to create a copy of a route
function Base.copy(route::Route)
    return Route(copy(route.visited_nodes), route.load, route.length, route.cost, route.duration, route.feasible, route.changed)
end

# Function to display the solution
function display_solution(pvrp_solution::PVRPSolution, instance::PVRPInstanceStruct, title::String)
    println(title)
    for day in sort(collect(keys(pvrp_solution.tourplan)))
        println("Day $day:")
        for (index, route) in enumerate(pvrp_solution.tourplan[day].routes)
            println("Route $index: $(route.visited_nodes)")
            println("Load: $(route.load), Length: $(route.length), Duration: $(route.duration), Feasible: $(route.feasible)")
        end
    end
end

# Function to save the solution to a YAML file
function save_solution_to_yaml(solution::PVRPSolution, filepath::String)
    # Convert the PVRPSolution to a dictionary
    solution_dict = Dict(
        "tourplan" => Dict(
            day => [
                Dict(
                    "visited_nodes" => route.visited_nodes,
                    "load" => route.load,
                    "length" => route.length,
                    "duration" => route.duration,
                    "feasible" => route.feasible,
                ) for route in solution.tourplan[day].routes
            ] for day in keys(solution.tourplan)
        ),
        "plan_length" => solution.plan_length,
        "plan_duration" => solution.plan_duration,
    )

    # Write the dictionary to a YAML file
    YAML.write_file(filepath, solution_dict)
    #println("Solution saved to $filepath")
end

# Function to load the solution from a YAML file
function load_solution_from_yaml(filepath::String)::PVRPSolution
    # Read the YAML file
    solution_dict = YAML.load_file(filepath)

    # Reconstruct the PVRPSolution
    tourplan = Dict(
        parse(Int, string(day)) => VRPSolution(
            [Route(route["visited_nodes"], route["load"], route["length"]) for route in solution_dict["tourplan"][day]],
            0.0,  # Placeholder for total_duration
            0.0   # Placeholder for total_length
        ) for day in keys(solution_dict["tourplan"])
    )
    
    # Return the reconstructed solution
    return PVRPSolution(tourplan, solution_dict["plan_length"], solution_dict["plan_duration"])
end

# Function to save run information to a YAML file
function save_run_info_to_yaml(seed::Int, runtime::Float64, cost::Float64, feasible::Bool, filepath::String)
    run_info = Dict(
        "seed" => seed,
        "runtime" => runtime,
        "cost" => cost,
        "feasible" => feasible,
    )
    YAML.write_file(filepath, run_info)
   #println("Run information saved to $filepath")
end

# Define a structure for the logbook
mutable struct VNSLogbook
    iteration::Vector{Int}
    best_solution_length::Vector{Float64}
    current_solution_length::Vector{Float64}
    best_feasible_solution_length::Vector{Float64}
    parameters::Dict{String, Vector{Float64}}  # Example: destroy_param
end

# Initialize the logbook
function initialize_logbook()::VNSLogbook
    return VNSLogbook([], [], [], [], Dict{String, Vector{Float64}}())
end

# Update the logbook with the current iteration data
function update_logbook!(
    logbook::VNSLogbook,
    iteration::Int,
    current_length::Float64,
    best_length::Float64,
    best_feasible_length::Float64;
    feasible::Bool = true
)
    push!(logbook.iteration, iteration)
    push!(logbook.current_solution_length, current_length)
    push!(logbook.best_solution_length, best_length)
    push!(logbook.best_feasible_solution_length, best_feasible_length)
end

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
    savefig(sols_p, joinpath(output_dir, "solution_evolution_plot.png"))

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

# Function to save the logbook to a YAML file
function save_logbook_to_yaml(logbook::VNSLogbook, filepath::String)
    # Convert the VNSLogbook to a dictionary
    logbook_dict = Dict(
        "iteration" => logbook.iteration,
        "best_solution_length" => logbook.best_solution_length,
        "current_solution_length" => logbook.current_solution_length,
        "best_feasible_solution_length" => logbook.best_feasible_solution_length,
        "parameters" => logbook.parameters
    )

    # Write the dictionary to a YAML file
    YAML.write_file(filepath, logbook_dict)
    #println("Logbook saved to $filepath")
end

# Function to recalculate the total plan length and duration of a PVRPSolution
function recalculate_plan_length!(solution::PVRPSolution)
    total_length = 0.0
    total_duration = 0.0
    for vrp_solution in values(solution.tourplan)
        for route in vrp_solution.routes
            total_length += route.length
            total_duration += route.duration
        end
    end
    solution.plan_length = total_length
    solution.plan_duration = total_duration
end

function calculate_kpis_with_treatment(solution::PVRPSolution, instance::PVRPInstanceStruct, region::Symbol, bring_participation::Float64, ev_share::Float64, average_idle_time_per_stop::Float64, compacting_energy::Float64, stops_per_compacting::Int, average_speed::Float64, treatment_distance::Float64, average_load::Float64, stop_energy::Float64, energy_per_km::Float64, idle_energy::Float64)::Dict{String, Any}
    # Default values based on region
    defaults = Dict(
        :urban => Dict(),
        :suburban => Dict(),
        :rural => Dict()
    )
    params = defaults[region]
    params["speed"] = average_speed
    params["load"] = average_load
    params["stop_energy"] = stop_energy
    params["energy_per_km"] = energy_per_km
    params["idle_energy"] = idle_energy

    # Initialize variables
    total_energy = 0.0
    total_stops = 0
    total_idle_time = 0.0
    total_distance = 0.0

    # Energy consumption by phases
    transport_energy = 0.0  # Ehaul
    collection_energy = 0.0  # Edrivecollect
    stop_energy_total = 0.0  # Estopcollect
    idle_energy_total = 0.0  # Idle Phase

    for (day, vrp_solution) in solution.tourplan
        for route in vrp_solution.routes
            # Collection phase: distance within the collection area
            collection_distance = route.length
            total_distance += collection_distance
            collection_energy += collection_distance * params["energy_per_km"]

            # Stops and energy consumption
            stops = length(route.visited_nodes) - 2  # Exclude depot
            total_stops += stops
            stop_energy_total += stops * params["stop_energy"]

            # Add compacting energy
            stop_energy_total += (stops / stops_per_compacting) * compacting_energy

            # Idle phase
            idle_time = stops * average_idle_time_per_stop * (1.0 - bring_participation)
            total_idle_time += idle_time
            idle_energy_total += idle_time * params["idle_energy"]

            # Transport phase: return trip to treatment facility
            transport_energy += treatment_distance * params["energy_per_km"]
        end
    end

    # Total energy
    total_energy = transport_energy + collection_energy + stop_energy_total + idle_energy_total

    # Energy consumption based on vehicle types
    ev_energy = total_energy * ev_share
    diesel_energy = total_energy * (1 - ev_share)

    # Conversion to specific units
    diesel_liters = diesel_energy / 36.0  # 1 liter of diesel = 36 MJ
    ev_mwh = ev_energy / 3600.0  # 1 MWh = 3600 MJ

    # Emissions (CO2 in kg)
    diesel_emissions = diesel_liters * 2.64  # kg CO2 per liter of diesel
    ev_emissions = ev_mwh * 0.0  # Assumption: 0 emissions for electric vehicles
    total_emissions = diesel_emissions + ev_emissions

    # Emissions by phases
    transport_emissions = transport_energy / 36.0 * 2.64
    collection_emissions = collection_energy / 36.0 * 2.64
    stop_emissions = stop_energy_total / 36.0 * 2.64
    idle_emissions = idle_energy_total / 36.0 * 2.64

    # Output as dictionary
    return Dict(
        "Total Energy (MJ)" => total_energy,
        "EV Consumption (MWh)" => ev_mwh,
        "Diesel Consumption (Liters)" => diesel_liters,
        "Total Emissions (kg CO2)" => total_emissions,
        "Total Distance (km)" => total_distance,
        "Total Stops" => total_stops,
        "Total Idle Time (h)" => total_idle_time,
        "Transport Phase (Ehaul)" => Dict("Energy (MJ)" => transport_energy, "Distance (km)" => treatment_distance * length(solution.tourplan), "Emissions (kg CO2)" => transport_emissions),
        "Collection Phase (Edrivecollect)" => Dict("Energy (MJ)" => collection_energy, "Distance (km)" => total_distance, "Emissions (kg CO2)" => collection_emissions),
        "Stop Phase (Estopcollect)" => Dict("Energy (MJ)" => stop_energy_total, "Stops" => total_stops, "Emissions (kg CO2)" => stop_emissions),
        "Idle Phase" => Dict("Energy (MJ)" => idle_energy_total, "Idle Time (h)" => total_idle_time, "Emissions (kg CO2)" => idle_emissions)
    )
end

function display_kpis(best_solution::PVRPSolution, instance::PVRPInstanceStruct, region::Symbol, bring_participation::Float64, ev_share::Float64, average_idle_time_per_stop::Float64, compacting_energy::Float64, stops_per_compacting::Int, average_speed::Float64, treatment_distance::Float64, average_load::Float64, stop_energy::Float64, energy_per_km::Float64, idle_energy::Float64)
    # Calculate KPIs
    println("\nCalculating KPIs with the following parameters:")
    println("Region: ", region)
    println("Bring Participation: ", bring_participation)
    println("EV Share: ", ev_share)
    println("Average Idle Time per Stop (h): ", average_idle_time_per_stop)
    println("Compacting Energy (MJ): ", compacting_energy)
    println("Stops per Compacting: ", stops_per_compacting)
    println("Average Speed (km/h): ", average_speed)
    println("Treatment Distance (km): ", treatment_distance)
    println("Average Load (tons): ", average_load)
    println("Stop Energy (MJ): ", stop_energy)
    println("Energy per km (MJ): ", energy_per_km)
    println("Idle Energy (MJ/h): ", idle_energy)
    println("====================================")
    kpis = calculate_kpis_with_treatment(
        best_solution, instance, region,
        bring_participation,
        ev_share,
        average_idle_time_per_stop,
        compacting_energy,
        stops_per_compacting,
        average_speed,
        treatment_distance,
        average_load,
        stop_energy,
        energy_per_km,
        idle_energy
    )
    println("\nKPIs Summary:")
    for (key, value) in kpis
        println(key, ": ", value)
    end

    # Print the portion of the total energy caused by each phase
    total_energy = kpis["Total Energy (MJ)"]
    transport_energy = kpis["Transport Phase (Ehaul)"]["Energy (MJ)"]
    collection_energy = kpis["Collection Phase (Edrivecollect)"]["Energy (MJ)"]
    stop_energy_total = kpis["Stop Phase (Estopcollect)"]["Energy (MJ)"]
    idle_energy_total = kpis["Idle Phase"]["Energy (MJ)"]

    println("\nEnergy Portion by Phase:")
    println("  Transport Phase: ", round(transport_energy / total_energy * 100, digits=2), "%")
    println("  Collection Phase: ", round(collection_energy / total_energy * 100, digits=2), "%")
    println("  Stop Phase: ", round(stop_energy_total / total_energy * 100, digits=2), "%")
    println("  Idle Phase: ", round(idle_energy_total / total_energy * 100, digits=2), "%")

    # Print the CO2 emissions for each phase
    transport_emissions = kpis["Transport Phase (Ehaul)"]["Emissions (kg CO2)"]
    collection_emissions = kpis["Collection Phase (Edrivecollect)"]["Emissions (kg CO2)"]
    stop_emissions = kpis["Stop Phase (Estopcollect)"]["Emissions (kg CO2)"]
    idle_emissions = kpis["Idle Phase"]["Emissions (kg CO2)"]

    println("\nCO2 Emissions by Phase (kg):")
    println("  Transport Phase: ", transport_emissions)
    println("  Collection Phase: ", collection_emissions)
    println("  Stop Phase: ", stop_emissions)
    println("  Idle Phase: ", idle_emissions)
end

function run_parameter_study(instance::PVRPInstanceStruct, best_solution::PVRPSolution, save_path::String)
    results = []

    bring_participations = [0.5, 0.7, 0.8, 0.9]  # Different values for bring participation
    ev_shares = [0.2, 0.4, 0.6, 0.8]  # Different values for EV share
    regions = [:urban, :suburban, :rural]  # Different regions
    compacting_energies = [8.0, 10.0, 12.0]  # Different values for compacting energy (MJ)
    stops_per_compactings = [4, 5, 6]  # Different values for stops per compacting
    average_speeds = [30.0, 40.0, 50.0]  # Different values for average speed (km/h)
    treatment_distances = [5.0, 10.0, 15.0]  # Different values for treatment distance (km)
    average_loads = [4.0, 5.0, 6.0]  # Different values for average load (tons)
    stop_energies = [2.0, 2.3, 2.5]  # Different values for stop energy (MJ)
    energies_per_km = [8.0, 9.0, 10.0]  # Different values for energy per km (MJ)
    idle_energies = [30.0, 36.0, 40.0]  # Different values for idle energy (MJ/h)

    for region in regions
        for bring_participation in bring_participations
            for ev_share in ev_shares
                for compacting_energy in compacting_energies
                    for stops_per_compacting in stops_per_compactings
                        for average_speed in average_speeds
                            for treatment_distance in treatment_distances
                                for average_load in average_loads
                                    for stop_energy in stop_energies
                                        for energy_per_km in energies_per_km
                                            for idle_energy in idle_energies
                                                # Calculate KPIs
                                                kpis = calculate_kpis_with_treatment(
                                                    best_solution, instance,
                                                    region,
                                                    bring_participation,
                                                    ev_share,
                                                    0.1,  # Assuming a fixed average idle time per stop
                                                    compacting_energy,
                                                    stops_per_compacting,
                                                    average_speed,
                                                    treatment_distance,
                                                    average_load,
                                                    stop_energy,
                                                    energy_per_km,
                                                    idle_energy
                                                )
                                                # Save results
                                                push!(results, Dict(
                                                    "Region" => region,
                                                    "Bring Participation" => bring_participation,
                                                    "EV Share" => ev_share,
                                                    "Compacting Energy (MJ)" => compacting_energy,
                                                    "Stops per Compacting" => stops_per_compacting,
                                                    "Average Speed (km/h)" => average_speed,
                                                    "Treatment Distance (km)" => treatment_distance,
                                                    "Average Load (tons)" => average_load,
                                                    "Stop Energy (MJ)" => stop_energy,
                                                    "Energy per km (MJ)" => energy_per_km,
                                                    "Idle Energy (MJ/h)" => idle_energy,
                                                    "Total Energy (MJ)" => kpis["Total Energy (MJ)"],
                                                    "Total Emissions (kg CO2)" => kpis["Total Emissions (kg CO2)"],
                                                    "Transport Phase Energy (MJ)" => kpis["Transport Phase (Ehaul)"]["Energy (MJ)"],
                                                    "Collection Phase Energy (MJ)" => kpis["Collection Phase (Edrivecollect)"]["Energy (MJ)"],
                                                    "Stop Phase Energy (MJ)" => kpis["Stop Phase (Estopcollect)"]["Energy (MJ)"],
                                                    "Idle Phase Energy (MJ)" => kpis["Idle Phase"]["Energy (MJ)"]
                                                ))
                                            end
                                        end
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
    end

    # Save results as YAML
    YAML.write_file(joinpath(save_path, "parameter_study_results.yaml"), results)
    #println("Parameter study results saved to $save_path/parameter_study_results.yaml")
end

function load_solution_and_calculate_kpis(filepath::String, instance::PVRPInstanceStruct, region::Symbol, bring_participation::Float64, ev_share::Float64, average_idle_time_per_stop::Float64, compacting_energy::Float64, stops_per_compacting::Int, average_speed::Float64, treatment_distance::Float64, average_load::Float64, stop_energy::Float64, energy_per_km::Float64, idle_energy::Float64)
    # Load the solution from the YAML file
    solution = load_solution_from_yaml(filepath)
    
    # Display the loaded solution
    display_solution(solution, instance, "Loaded Solution")

    # Calculate and display KPIs
    display_kpis(solution, instance, region, bring_participation, ev_share, average_idle_time_per_stop, compacting_energy, stops_per_compacting, average_speed, treatment_distance, average_load, stop_energy, energy_per_km, idle_energy)
end

end # module

