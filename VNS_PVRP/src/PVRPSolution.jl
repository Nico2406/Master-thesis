module Solution

using ..PVRPInstance: PVRPInstanceStruct, Node, plot_instance, get_fitting_layout
using Plots
using YAML: write_file, read_file

export Route, PVRPSolution, VRPSolution, plot_solution, plot_solution!, validate_route, validate_solution, recalculate_route!, remove_segment!, insert_segment, display_solution, save_solution_to_yaml, load_solution_from_yaml, save_run_info_to_yaml

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
    for i in 1:(length(route.visited_nodes) - 1)
        from_node = route.visited_nodes[i]
        to_node = route.visited_nodes[i + 1]
        if from_node < 0 || from_node >= size(instance.distance_matrix, 1) || to_node < 0 || to_node >= size(instance.distance_matrix, 2)
            continue
        end
        route.length += instance.distance_matrix[from_node + 1, to_node + 1]
        if to_node != 0  # Exclude depot
            # Adjust the index to access the correct node data
            node_index = to_node + 1
            route.load += instance.nodes[node_index].demand / instance.nodes[node_index].frequency
        end
    end
    # Ensure the route length is correctly updated
    route.length += instance.distance_matrix[route.visited_nodes[end] + 1, 1]
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

# Function to validate a route
function validate_route(route::Route, instance::PVRPInstanceStruct)::Bool
    return validate_route(route, instance, 1)  # Default to day 1 if day is not provided
end

# Function to validate a route for a specific day
function validate_route(route::Route, instance::PVRPInstanceStruct, day::Int)::Bool
    recalculate_route!(route, instance)
    
    if route.load > instance.vehicleload
        println("Constraint violated: Load exceeds vehicle capacity.")
        return false
    end
    
    if route.length + instance.distance_matrix[route.visited_nodes[end] + 1, 1] > instance.maximumrouteduration
        println("Constraint violated: Route length exceeds maximum route duration.")
        return false
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
    
    return true
end

# Function to validate the entire solution
function validate_solution(sol::PVRPSolution, inst::PVRPInstanceStruct)::Bool
    for (day, vrp_solution) in sol.tourplan
        for route in vrp_solution.routes
            if !validate_route(route, inst, day)
                return false
            end
        end
    end
    return true
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
            println("Route $index: $(route.visited_nodes), Load: $(route.load), Length: $(route.length)")
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
    println("Solution saved to $filepath")
end

# Function to load the solution from a YAML file
function load_solution_from_yaml(filepath::String)::PVRPSolution
    # Read the YAML file
    solution_dict = YAML.read_file(filepath)

    # Reconstruct the PVRPSolution
    tourplan = Dict(
        parse(Int, day) => VRPSolution(
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
    println("Run information saved to $filepath")
end

end # module

