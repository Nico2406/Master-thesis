module Shaking

using ..PVRPInstance: PVRPInstanceStruct, Node, convert_binary_int
using ..Solution: PVRPSolution, VRPSolution, Route, recalculate_route!, remove_segment!, insert_segment!
using Random: shuffle!
using Plots

export shaking!, move!

function move!(route1::Route, route2::Route, start_idx::Int, segment_length::Int, instance::PVRPInstanceStruct, day::Int)::Float64
    if start_idx < 1 || start_idx + segment_length - 1 > length(route1.visited_nodes)
        error("Invalid segment range: out of bounds in route1.")
    end

    segment = route1.visited_nodes[start_idx:(start_idx + segment_length - 1)]

    for node in segment
        if node != 0 && !instance.nodes[node + 1].initialvisitcombination[day]
            error("Node $node cannot be assigned to day $day with the current visit combination.")
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

    delta_insert = insert_segment!(route2, best_position, segment, instance)

    route1.changed = true
    route2.changed = true

    return delta_remove + delta_insert
end

function shaking!(solution::PVRPSolution, instance::PVRPInstanceStruct, day::Int)
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

    try
        original_node_count = sum(length(route.visited_nodes) for route in solution.tourplan[day].routes)
        delta = move!(route1, route2, start_idx, segment_length, instance, day)
        if delta == Inf
            return 0.0
        end

        route1.changed = true
        route2.changed = true

        # Sicherstellen, dass keine doppelten Routen entstehen
        # unique_routes = Set{Vector{Int}}()
        # for route in solution.tourplan[day].routes
        #     if route.visited_nodes in unique_routes
        #         println("Warning: Duplicate route detected during shaking on day $day.")
        #     else
        #         push!(unique_routes, deepcopy(route.visited_nodes))
        #     end
        # end

        # Routen aktualisieren und leere entfernen
        solution.tourplan[day].routes = filter(route -> !(isempty(route.visited_nodes) || route.visited_nodes == [0, 0]), solution.tourplan[day].routes)

        # Knotenanzahl prüfen
        # new_node_count = sum(length(route.visited_nodes) for route in solution.tourplan[day].routes)
        # if original_node_count != new_node_count
        #     println("Warning: Node count mismatch detected after shaking on day $day. This might be caused by filtering.")
        # end

        return delta
    catch e
        # println("Error during shaking on day $day: $e")
        return 0.0
    end
end

end # module
