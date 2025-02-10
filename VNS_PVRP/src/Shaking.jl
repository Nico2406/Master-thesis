module Shaking

using ..PVRPInstance: PVRPInstanceStruct, Node, convert_binary_int
using ..Solution: PVRPSolution, VRPSolution, Route, recalculate_route!, remove_segment!, insert_segment!, validate_solution, recalculate_plan_length!
using Random: shuffle, shuffle!, rand
using Plots

export shaking!, move!, change_visit_combinations!, change_visit_combinations_sequences!, change_visit_combinations_sequences_no_improvement!

function move!(solution::PVRPSolution, instance::PVRPInstanceStruct, day::Int, k::Int)::Float64
    if isempty(solution.tourplan[day].routes) || length(solution.tourplan[day].routes) < 2
        return 0.0
    end

    route_ids = shuffle(collect(1:length(solution.tourplan[day].routes)))
    route1 = solution.tourplan[day].routes[route_ids[1]]
    route2 = solution.tourplan[day].routes[route_ids[2]]

    if length(route1.visited_nodes) <= 2
        return 0.0
    end

    segment_length = min(k, rand(1:3))
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
        insert_segment!(route1, start_idx, segment, instance)
        return Inf  # Return a high cost to indicate failure
    end

    insert_segment!(route2, best_position, segment, instance)
    route1.changed = true
    route2.changed = true

    recalculate_route!(route1, instance)
    recalculate_route!(route2, instance)
    recalculate_plan_length!(solution)

    return delta_remove + best_delta
end

function change_visit_combinations!(solution::PVRPSolution, instance::PVRPInstanceStruct, k::Int)::Float64
    total_delta = 0.0

    # Collect all nodes in the solution (excluding depot)
    all_nodes = unique(vcat([route.visited_nodes[2:end-1] for vrp_solution in values(solution.tourplan) for route in vrp_solution.routes]...))

    # Bestimmen der Anzahl der zu ändernden Nodes abhängig von k
    num_nodes = length(all_nodes)
    max_changes = min(k, num_nodes)  # Maximal k Kunden ändern
    selected_nodes = shuffle(all_nodes)[1:max_changes]

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
    percentage = rand(20:30) / 100
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

function cross_exchange!(solution::PVRPSolution, instance::PVRPInstanceStruct, day::Int, k::Int)::Float64
    if isempty(solution.tourplan[day].routes) || length(solution.tourplan[day].routes) < 2
        return 0.0
    end

    route_ids = shuffle(collect(1:length(solution.tourplan[day].routes)))
    route1 = solution.tourplan[day].routes[route_ids[1]]
    route2 = solution.tourplan[day].routes[route_ids[2]]

    if length(route1.visited_nodes) <= 2 || length(route2.visited_nodes) <= 2
        return 0.0
    end

    segment_length = min(k, rand(1:6))  # Maximal 6 Kunden pro Segment
    if length(route1.visited_nodes) - segment_length < 2 || length(route2.visited_nodes) - segment_length < 2
        return 0.0
    end

    start_idx1 = rand(2:(length(route1.visited_nodes) - segment_length))
    start_idx2 = rand(2:(length(route2.visited_nodes) - segment_length))

    segment1 = route1.visited_nodes[start_idx1:(start_idx1 + segment_length - 1)]
    segment2 = route2.visited_nodes[start_idx2:(start_idx2 + segment_length - 1)]

    for node in segment1
        if node != 0 && !instance.nodes[node + 1].initialvisitcombination[day]
            return 0.0  # Skip if the node cannot be assigned to the day
        end
    end
    for node in segment2
        if node != 0 && !instance.nodes[node + 1].initialvisitcombination[day]
            return 0.0  # Skip if the node cannot be assigned to the day
        end
    end

    delta_remove1 = remove_segment!(route1, start_idx1, segment_length, instance)
    delta_remove2 = remove_segment!(route2, start_idx2, segment_length, instance)

    best_delta = Inf
    best_position1, best_position2 = -1, -1

    for insert_idx1 in 2:length(route1.visited_nodes)
        for insert_idx2 in 2:length(route2.visited_nodes)
            temp_route1 = deepcopy(route1)
            temp_route2 = deepcopy(route2)
            delta_insert1 = insert_segment!(temp_route1, insert_idx1, segment2, instance)
            delta_insert2 = insert_segment!(temp_route2, insert_idx2, segment1, instance)
            
            if temp_route1.load <= instance.vehicleload && temp_route2.load <= instance.vehicleload && (delta_insert1 + delta_insert2 < best_delta)
                best_delta = delta_insert1 + delta_insert2
                best_position1, best_position2 = insert_idx1, insert_idx2
            end
        end
    end

    if best_position1 == -1 || best_position2 == -1
        insert_segment!(route1, start_idx1, segment1, instance)
        insert_segment!(route2, start_idx2, segment2, instance)
        return Inf  # Return a high cost to indicate failure
    end

    insert_segment!(route1, best_position1, segment2, instance)
    insert_segment!(route2, best_position2, segment1, instance)
    
    route1.changed = true
    route2.changed = true

    recalculate_route!(route1, instance)
    recalculate_route!(route2, instance)
    recalculate_plan_length!(solution)

    return delta_remove1 + delta_remove2 + best_delta
end

function shaking!(solution::PVRPSolution, instance::PVRPInstanceStruct, k::Int)::Float64
    if isempty(solution.tourplan)
        return 0.0
    end

    delta = 0.0

    if 1 <= k <= 6
        # Change visit combinations operation
        delta = change_visit_combinations!(solution, instance, k)
    elseif 7 <= k <= 9
        # Move operation on a random day
        day = rand(keys(solution.tourplan))
        delta = move!(solution, instance, day, k)
    elseif 10 <= k <= 15
        # Placeholder for Cross-Exchange operation
        day = rand(keys(solution.tourplan))
        delta = cross_exchange!(solution, instance, day, k)
    end

    # Recalculate the total plan length
    recalculate_plan_length!(solution)

    return delta
end

end # module Shaking
