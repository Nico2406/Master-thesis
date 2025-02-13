module ConstructionHeuristics

using ..PVRPInstance: PVRPInstanceStruct, convert_binary_int
using ..Solution: PVRPSolution, VRPSolution, Route, recalculate_route!, remove_segment!, plot_solution, validate_route
using Plots

export nearest_neighbor

# Initialize the visit combinations for each node in the instance
function initialization(instance::PVRPInstanceStruct)
    for node in instance.nodes[2:end]  # Skip the depot (first node)
        chosen_combination = rand(node.visitcombinations)
        node.initialvisitcombination = chosen_combination
        node.initialvisitcombinationint = convert_binary_int(chosen_combination)
        #println("Node $(node.id) initial visit combination: $(node.initialvisitcombination) with frequency $(node.frequency)")
    end
end

# Find the nearest unvisited node from the current node for a given day
function find_nearest_node(current_node_index::Int, instance::PVRPInstanceStruct, visited::Set{Int}, day::Int)
    nearest_node_index = nothing
    nearest_distance = Inf

    for i in 2:length(instance.nodes)  # Skip the depot (index 1)
        if !(i in visited) && instance.nodes[i].initialvisitcombination[day]
            distance = instance.distance_matrix[current_node_index, i]
            if distance < nearest_distance
                nearest_distance = distance
                nearest_node_index = i
            end
        end
    end
    
    return nearest_node_index
end

# Create a big route for a given day by visiting all nodes that need to be visited on that day
function create_big_route(instance::PVRPInstanceStruct, day::Int)::Route
    big_route = Route([instance.nodes[1].id], 0.0, 0.0, 0.0, 0.0, false, false, 0.0)  # Start with the depot (id of the first node)
    current_node_index = 1  # Depot index
    visited = Set([current_node_index])
    unvisited_customers = Set{Int}()

    # Add each customer to unvisited_customers if initialvisitcombination is true for the given day
    for node in instance.nodes[2:end]  # Skip the depot (first node)
        if node.initialvisitcombination[day] == 1
            push!(unvisited_customers, node.id)
        end
    end

    # Visit nodes until all required nodes are visited
    while !isempty(unvisited_customers)
        nearest_node_index = find_nearest_node(current_node_index, instance, visited, day)
        if nearest_node_index === nothing
            break
        end

        AddNodeToRoute!(big_route, instance.nodes[nearest_node_index].id)
        push!(visited, nearest_node_index)
        delete!(unvisited_customers, instance.nodes[nearest_node_index].id)
        current_node_index = nearest_node_index
    end
    
    # End the route by returning to the depot
    if big_route.visited_nodes[end] != instance.nodes[1].id
        AddNodeToRoute!(big_route, instance.nodes[1].id)
    end
    
    # Recalculate the route length/duration after construction
    recalculate_route!(big_route, instance)
    
    return big_route
end

# Split a big route into multiple routes 
function split_routes(big_route::Route, instance::PVRPInstanceStruct, day::Int)::VRPSolution
    vrp_solution = VRPSolution(Vector{Route}(), 0.0, 0.0)

    # Divide the nodes equally among the number of vehicles
    total_nodes = big_route.visited_nodes[2:end-1]  # Exclude depot at start and end
    nodes_per_vehicle = ceil(Int, length(total_nodes) / instance.numberofvehicles)

    # println("Day $day: Starting split_routes with ", length(total_nodes), " nodes to process.")

    for i in 1:instance.numberofvehicles
        start_index = (i - 1) * nodes_per_vehicle + 1
        end_index = min(i * nodes_per_vehicle, length(total_nodes))

        if start_index > length(total_nodes)
            break
        end

        # Create a route for the current vehicle
        route_nodes = [instance.nodes[1].id; total_nodes[start_index:end_index]; instance.nodes[1].id]
        route = Route(route_nodes, 0.0, 0.0, 0.0, 0.0,false, false, 0.0)
        recalculate_route!(route, instance)

        push!(vrp_solution.routes, route)
        # println("Route $i: ", route.visited_nodes, ", Load: ", route.load, ", Length: ", route.length)
    end

    # println("Day $day: Completed split_routes. Total routes created: ", length(vrp_solution.routes))
    return vrp_solution
end

# Construct a solution using the nearest neighbor heuristic
function nearest_neighbor(instance::PVRPInstanceStruct)::PVRPSolution
    initialization(instance)

    tourplan = Dict{Int, VRPSolution}()
    total_length = 0.0
    total_duration = 0.0
    big_routes = Dict{Int, Route}()

    # Create big routes for each day
    for day in 1:instance.numberofdays
        big_route = create_big_route(instance, day)
        big_routes[day] = big_route

        """
        # Validate the big route using the function from PVRPSolution
        if validate_route(big_route, instance, day)
            # println("Big route for day $day is valid")
        else
            # println("Big route for day $day is invalid")
        end
        """
    end

    # Split big routes into feasible routes and add to the tour plan
    for day in 1:instance.numberofdays
        big_route = big_routes[day]
        vrp_solution = split_routes(big_route, instance, day)
        
        # Recalculate each route in the VRP solution to ensure validity
        for route in vrp_solution.routes
            recalculate_route!(route, instance)
        end
        
        tourplan[day] = vrp_solution
        total_length += sum(route.length for route in vrp_solution.routes)
        total_duration += sum(route.duration for route in vrp_solution.routes)
    end

    return PVRPSolution(tourplan, total_length, total_duration)
end

# Add a node to the route
function AddNodeToRoute!(route::Route, node_id::Int)
    push!(route.visited_nodes, node_id)
end

end # module
