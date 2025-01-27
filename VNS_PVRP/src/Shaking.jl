module Shaking

using ..PVRPInstance: PVRPInstanceStruct, Node, convert_binary_int
using ..Solution: PVRPSolution, VRPSolution, Route, recalculate_route!, remove_segment!, insert_segment!, validate_solution
using Random: shuffle, shuffle!, rand
using Plots

export shaking!, move!, change_visit_combinations!

function move!(solution::PVRPSolution, instance::PVRPInstanceStruct, day::Int)::Float64
    if isempty(solution.tourplan[day].routes)
        return 0.0
    end

    route_ids = collect(1:length(solution.tourplan[day].routes))
    if length(route_ids) < 2
        return 0.0
    end
    shuffle!(route_ids)

    route1 = solution.tourplan[day].routes[route_ids[1]]
    route2 = solution.tourplan[day].routes[route_ids[2]]

    if length(route1.visited_nodes) <= 2
        return 0.0
    end

    segment_length = rand(1:3)
    if length(route1.visited_nodes) - segment_length < 2
        return 0.0
    end
    start_idx = rand(2:(length(route1.visited_nodes) - segment_length))

    segment = route1.visited_nodes[start_idx:(start_idx + segment_length - 1)]

    for node in segment
        if node != 0 && !instance.nodes[node + 1].initialvisitcombination[day]
            return 0.0  # Skip if the node cannot be assigned to the day
        end
    end

    delta_remove = remove_segment!(route1, start_idx, segment_length, instance)

    best_delta = Inf
    best_position = -1

    for insert_idx in 2:length(route2.visited_nodes)
        temp_route = deepcopy(route2)
        delta_insert = insert_segment!(temp_route, insert_idx, segment, instance)
        if temp_route.load <= instance.vehicleload && delta_insert < best_delta
            best_delta = delta_insert
            best_position = insert_idx
        end
    end

    if best_position == -1
        # Revert the removal
        insert_segment!(route1, start_idx, segment, instance)
        return Inf  # Return a high cost to indicate failure
    end

    insert_segment!(route2, best_position, segment, instance)

    route1.changed = true
    route2.changed = true

    # Update routes and remove empty ones
    solution.tourplan[day].routes = filter(route -> !(isempty(route.visited_nodes) || route.visited_nodes == [0, 0]), solution.tourplan[day].routes)

    return delta_remove + best_delta
end

function change_visit_combinations!(solution::PVRPSolution, instance::PVRPInstanceStruct)::Float64
    total_delta = 0.0
    # println("Starting change_visit_combinations!")

    # Collect all nodes in the solution (excluding depot)
    all_nodes = unique(vcat([route.visited_nodes[2:end-1] for vrp_solution in values(solution.tourplan) for route in vrp_solution.routes]...))
    # println("All nodes in the solution: $all_nodes")
    
    num_changes = rand(1:min(3, length(all_nodes)))  # Random number of nodes to update
    selected_nodes = shuffle(all_nodes)[1:num_changes]
    # println("Selected nodes for visit combination change: $selected_nodes")

    for node in selected_nodes
        current_combination = instance.nodes[node + 1].initialvisitcombination
        all_combinations = instance.nodes[node + 1].visitcombinations
        valid_combinations = filter(vc -> vc != current_combination, all_combinations)

        if isempty(valid_combinations)
            # println("Node $node: No valid combinations other than the current one. Skipping.")
            continue
        end

        # Select a new valid combination
        new_combination = first(shuffle(valid_combinations))
        # println("Node $node: Changing visit combination from $current_combination to $new_combination")
        instance.nodes[node + 1].initialvisitcombination = new_combination

        # Update routes: Remove node from old days
        for (day, vrp_solution) in solution.tourplan
            if current_combination[day] && !new_combination[day]
                for route in vrp_solution.routes
                    idx = findfirst(x -> x == node, route.visited_nodes)
                    if idx !== nothing
                        delta = remove_segment!(route, idx, 1, instance)
                        total_delta += delta
                        route.changed = true  # Mark the route as changed
                        # println("Node $node removed from Day $day. Delta: $delta")
                    end
                end
            end
        end

        # Insert node into new days
        for (day, vrp_solution) in solution.tourplan
            if new_combination[day] && !current_combination[day]
                # Check if the node is already present on the day
                if any(node in route.visited_nodes for route in vrp_solution.routes)
                    continue
                end

                best_delta = Inf
                best_route = nothing
                best_position = -1

                for route in vrp_solution.routes
                    if route.load + instance.nodes[node + 1].demand / instance.nodes[node + 1].frequency <= instance.vehicleload && 
                       route.length + instance.distance_matrix[route.visited_nodes[end] + 1, node + 1] + 
                       instance.distance_matrix[node + 1, route.visited_nodes[1] + 1] + 
                       instance.nodes[node + 1].demand / instance.nodes[node + 1].frequency <= instance.maximumrouteduration

                        for insert_idx in 2:length(route.visited_nodes)
                            temp_route = deepcopy(route)
                            delta = insert_segment!(temp_route, insert_idx, [node], instance)
                            if delta < best_delta
                                best_delta = delta
                                best_route = route
                                best_position = insert_idx
                            end
                        end
                    end
                end

                if best_route !== nothing
                    delta = insert_segment!(best_route, best_position, [node], instance)
                    total_delta += delta
                    best_route.changed = true  # Mark the route as changed
                    # println("Node $node inserted into Day $day at position $best_position. Delta: $delta")
                else
                    # Create a new route if no existing route can accommodate the node
                    new_route = Route([instance.nodes[1].id, node, instance.nodes[1].id], 
                                      instance.nodes[node + 1].demand / instance.nodes[node + 1].frequency, 
                                      0.0, 0.0, 0.0, true, false)
                    recalculate_route!(new_route, instance)
                    push!(vrp_solution.routes, new_route)
                    total_delta += new_route.length
                    new_route.changed = true  # Mark the new route as changed
                    # println("Node $node inserted into a new route for Day $day. Delta: $(new_route.length)")
                end
            end
        end
    end

    # Remove empty routes
    for (day, vrp_solution) in solution.tourplan
        vrp_solution.routes = filter(route -> !(isempty(route.visited_nodes) || route.visited_nodes == [0, 0]), vrp_solution.routes)
    end

    # println("Change visit combinations completed. Total delta: $total_delta")
    return total_delta
end

function shaking!(solution::PVRPSolution, instance::PVRPInstanceStruct)::Float64
    if isempty(solution.tourplan)
        return 0.0
    end

    if rand(Bool)
        # Perform move operation on a random day
        day = rand(keys(solution.tourplan))
        return move!(solution, instance, day)
    else
        # Perform change visit combinations operation
        return change_visit_combinations!(solution, instance)
    end
end

end # module
