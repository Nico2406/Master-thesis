module Shaking

using ..PVRPInstance: PVRPInstanceStruct, Node, convert_binary_int
using ..Solution: PVRPSolution, VRPSolution, Route, recalculate_route!, remove_segment!, insert_segment!, validate_solution, recalculate_plan_length!
using Random: shuffle, shuffle!, rand
using Plots

export shaking!, move!, change_visit_combinations!, change_visit_combinations_sequences!, change_visit_combinations_sequences_no_improvement!

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

    # we update the route lengths
    recalculate_route!(route1, instance)
    recalculate_route!(route2, instance)

    # update the total plan length
    recalculate_plan_length!(solution)

    return delta_remove + best_delta
end

function change_visit_combinations!(solution::PVRPSolution, instance::PVRPInstanceStruct)::Float64
    total_delta = 0.0

    # Collect all nodes in the solution (excluding depot)
    all_nodes = unique(vcat([route.visited_nodes[2:end-1] for vrp_solution in values(solution.tourplan) for route in vrp_solution.routes]...))

    # Determine the number of nodes to change (5-10% of all nodes)
    num_nodes = length(all_nodes)
    percentage = rand(5:10) / 100
    num_changes = max(1, round(Int, percentage * num_nodes))  # Ensure at least one node is selected
    selected_nodes = shuffle(all_nodes)[1:num_changes]

    for node in selected_nodes
        current_combination = instance.nodes[node + 1].initialvisitcombination
        all_combinations = instance.nodes[node + 1].visitcombinations
        valid_combinations = filter(vc -> vc != current_combination, all_combinations)

        if isempty(valid_combinations)
            continue
        end

        # Select a new valid combination
        new_combination = first(shuffle(valid_combinations))
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
                    end
                end
            end
        end

        # Insert node into new days
        for (day, vrp_solution) in solution.tourplan
            if new_combination[day] && !current_combination[day]
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
                else
                    new_route = Route([instance.nodes[1].id, node, instance.nodes[1].id], 
                                      instance.nodes[node + 1].demand / instance.nodes[node + 1].frequency, 
                                      0.0, 0.0, 0.0, true, false)
                    recalculate_route!(new_route, instance)
                    push!(vrp_solution.routes, new_route)
                    total_delta += new_route.length
                    new_route.changed = true  # Mark the new route as changed
                end
            end
        end
    end

    # Remove empty routes
    for (day, vrp_solution) in solution.tourplan
        vrp_solution.routes = filter(route -> !(isempty(route.visited_nodes) || route.visited_nodes == [0, 0]), vrp_solution.routes)
    end

    return total_delta
end

function change_visit_combinations_sequences!(solution::PVRPSolution, instance::PVRPInstanceStruct)::Float64
    total_delta = 0.0
    all_nodes = unique(vcat([route.visited_nodes[2:end-1] for vrp_solution in values(solution.tourplan) for route in vrp_solution.routes]...))
    num_nodes = length(all_nodes)
    percentage = rand(5:10) / 100
    num_changes = max(1, round(Int, percentage * num_nodes))
    selected_nodes = shuffle(all_nodes)[1:num_changes]

    for node in selected_nodes
        current_combination = instance.nodes[node + 1].initialvisitcombination
        all_combinations = instance.nodes[node + 1].visitcombinations
        valid_combinations = filter(vc -> vc != current_combination, all_combinations)

        if isempty(valid_combinations)
            continue
        end

        new_combination = first(shuffle(valid_combinations))
        instance.nodes[node + 1].initialvisitcombination = new_combination

        for (day, vrp_solution) in solution.tourplan
            if current_combination[day] && !new_combination[day]
                for route in vrp_solution.routes
                    idx = findfirst(x -> x == node, route.visited_nodes)
                    if idx !== nothing
                        segment_length = rand(1:min(6, length(route.visited_nodes) - idx))
                        delta = remove_segment!(route, idx, segment_length, instance)
                        total_delta += delta
                        route.changed = true
                    end
                end
            end
        end

        for (day, vrp_solution) in solution.tourplan
            if new_combination[day] && !current_combination[day]
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
                    best_route.changed = true
                else
                    new_route = Route([instance.nodes[1].id, node, instance.nodes[1].id], 
                                      instance.nodes[node + 1].demand / instance.nodes[node + 1].frequency, 
                                      0.0, 0.0, 0.0, true, false)
                    recalculate_route!(new_route, instance)
                    push!(vrp_solution.routes, new_route)
                    total_delta += new_route.length
                    new_route.changed = true
                end
            end
        end
    end

    for (day, vrp_solution) in solution.tourplan
        vrp_solution.routes = filter(route -> !(isempty(route.visited_nodes) || route.visited_nodes == [0, 0]), vrp_solution.routes)
    end

    # Ensure all nodes are visited
    visited_nodes = unique(vcat([route.visited_nodes[2:end-1] for vrp_solution in values(solution.tourplan) for route in vrp_solution.routes]...))
    missing_nodes = setdiff(all_nodes, visited_nodes)

    for node in missing_nodes
        for (day, vrp_solution) in solution.tourplan
            if instance.nodes[node + 1].initialvisitcombination[day]
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
                    best_route.changed = true
                else
                    new_route = Route([instance.nodes[1].id, node, instance.nodes[1].id], 
                                      instance.nodes[node + 1].demand / instance.nodes[node + 1].frequency, 
                                      0.0, 0.0, 0.0, true, false)
                    recalculate_route!(new_route, instance)
                    push!(vrp_solution.routes, new_route)
                    total_delta += new_route.length
                    new_route.changed = true
                end
            end
        end
    end

    return total_delta
end

function change_visit_combinations_sequences_no_improvement!(solution::PVRPSolution, instance::PVRPInstanceStruct)::Float64
    total_delta = 0.0
    all_nodes = unique(vcat([route.visited_nodes[2:end-1] for vrp_solution in values(solution.tourplan) for route in vrp_solution.routes]...))
    num_nodes = length(all_nodes)
    percentage = rand(10:15) / 100
    num_changes = max(1, round(Int, percentage * num_nodes))
    selected_nodes = shuffle(all_nodes)[1:num_changes]

    for node in selected_nodes
        current_combination = instance.nodes[node + 1].initialvisitcombination
        all_combinations = instance.nodes[node + 1].visitcombinations
        valid_combinations = filter(vc -> vc != current_combination, all_combinations)

        if isempty(valid_combinations)
            continue
        end

        new_combination = first(shuffle(valid_combinations))
        instance.nodes[node + 1].initialvisitcombination = new_combination

        for (day, vrp_solution) in solution.tourplan
            if current_combination[day] && !new_combination[day]
                for route in vrp_solution.routes
                    idx = findfirst(x -> x == node, route.visited_nodes)
                    if idx !== nothing
                        segment_length = rand(1:min(6, length(route.visited_nodes) - idx))
                        delta = remove_segment!(route, idx, segment_length, instance)
                        total_delta += delta
                        route.changed = true
                    end
                end
            end
        end

        for (day, vrp_solution) in solution.tourplan
            if new_combination[day] && !current_combination[day]
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
                    best_route.changed = true
                else
                    new_route = Route([instance.nodes[1].id, node, instance.nodes[1].id], 
                                      instance.nodes[node + 1].demand / instance.nodes[node + 1].frequency, 
                                      0.0, 0.0, 0.0, true, false)
                    recalculate_route!(new_route, instance)
                    push!(vrp_solution.routes, new_route)
                    total_delta += new_route.length
                    new_route.changed = true
                end
            end
        end
    end

    for (day, vrp_solution) in solution.tourplan
        vrp_solution.routes = filter(route -> !(isempty(route.visited_nodes) || route.visited_nodes == [0, 0]), vrp_solution.routes)
    end

    # Ensure all nodes are visited
    visited_nodes = unique(vcat([route.visited_nodes[2:end-1] for vrp_solution in values(solution.tourplan) for route in vrp_solution.routes]...))
    missing_nodes = setdiff(all_nodes, visited_nodes)

    for node in missing_nodes
        for (day, vrp_solution) in solution.tourplan
            if instance.nodes[node + 1].initialvisitcombination[day]
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
                    best_route.changed = true
                else
                    new_route = Route([instance.nodes[1].id, node, instance.nodes[1].id], 
                                      instance.nodes[node + 1].demand / instance.nodes[node + 1].frequency, 
                                      0.0, 0.0, 0.0, true, false)
                    recalculate_route!(new_route, instance)
                    push!(vrp_solution.routes, new_route)
                    total_delta += new_route.length
                    new_route.changed = true
                end
            end
        end
    end

    return total_delta
end

function shaking!(solution::PVRPSolution, instance::PVRPInstanceStruct)::Float64
    if isempty(solution.tourplan)
        return 0.0
    end

    choice = rand(1:3)
    delta = 0.0
    if choice == 1
        # Perform move operation on a random day
        day = rand(keys(solution.tourplan))
        delta = move!(solution, instance, day)
    elseif choice == 2
        # Perform change visit combinations operation
        delta = change_visit_combinations!(solution, instance)
    else
        # Perform change visit combinations sequences operation
        delta = change_visit_combinations_sequences!(solution, instance)
    end

    # Recalculate the total plan length
    recalculate_plan_length!(solution)


    return delta
end

end # module
