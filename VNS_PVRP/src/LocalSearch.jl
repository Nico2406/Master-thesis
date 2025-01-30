module LocalSearch

using ..PVRPInstance: PVRPInstanceStruct
using ..Solution: VRPSolution, Route, recalculate_route!
using Plots

export local_search!, two_opt_first_improvement!, two_opt_first_improvement_1iteration!

# Perform local search on the entire solution
function local_search!(sol::VRPSolution, inst::PVRPInstanceStruct, method::String, max_iterations::Int)::Number
    Δ = 0

    for route in sol.routes
        Δ += local_search!(route, inst, method, max_iterations)
    end

    sol.total_length += Δ
    sol.total_duration += Δ

    return Δ
end

# Perform local search on a single route
function local_search!(route::Route, inst::PVRPInstanceStruct, method::String, max_iterations::Int)::Number
    Δ = 0

    if method == "2opt-first"
        Δ += two_opt_first_improvement!(route, inst, max_iterations)
    elseif method == "swap-first"
        Δ += swap_first_improvement!(route, inst, max_iterations)
    elseif method == "reinsert-first"
        Δ += reinsert_first_improvement!(route, inst, max_iterations)
    else
        throw(ArgumentError("local_search! in Route.jl: method '$method' is not implemented"))
    end

    route.changed = false

    return Δ
end

# Apply the 2-opt first improvement heuristic to a route
function two_opt_first_improvement!(route::Route, inst::PVRPInstanceStruct, max_iterations::Int=1000)::Number
    Δ, totalΔ, iterations = -1, 0, 0
    while Δ != 0 && iterations < max_iterations
        Δ = two_opt_first_improvement_1iteration!(route, inst)
        totalΔ += Δ
        iterations += 1
    end
    return totalΔ
end

# Perform one iteration of the 2-opt first improvement heuristic
function two_opt_first_improvement_1iteration!(route::Route, inst::PVRPInstanceStruct)::Number
    routelen = length(route.visited_nodes)
    for i in 2:(routelen - 2)
        for j in (i + 1):(routelen - 1)
            Δ = inst.distance_matrix[route.visited_nodes[i - 1] + 1, route.visited_nodes[j] + 1] +
                inst.distance_matrix[route.visited_nodes[i] + 1, route.visited_nodes[j + 1] + 1] -
                inst.distance_matrix[route.visited_nodes[i - 1] + 1, route.visited_nodes[i] + 1] -
                inst.distance_matrix[route.visited_nodes[j] + 1, route.visited_nodes[j + 1] + 1]
            if Δ < 0
                route.visited_nodes = vcat(route.visited_nodes[1:(i - 1)], reverse(route.visited_nodes[i:j]), route.visited_nodes[(j + 1):end])
                recalculate_route!(route, inst)
                return Δ
            end
        end
    end
    return 0
end

# Apply the swap first improvement heuristic to a route
function swap_first_improvement!(route::Route, inst::PVRPInstanceStruct, max_iterations::Int=1000)::Number
    Δ, totalΔ, iterations = -1, 0, 0

    while Δ != 0 && iterations < max_iterations
        Δ = 0
        for i in 2:(length(route.visited_nodes) - 1)
            for j in (i + 1):(length(route.visited_nodes) - 1)
                # Calculate the change in distance if nodes i and j are swapped
                Δ_swap = inst.distance_matrix[route.visited_nodes[i - 1] + 1, route.visited_nodes[j] + 1] +
                         inst.distance_matrix[route.visited_nodes[i] + 1, route.visited_nodes[j + 1] + 1] -
                         inst.distance_matrix[route.visited_nodes[i - 1] + 1, route.visited_nodes[i] + 1] -
                         inst.distance_matrix[route.visited_nodes[j] + 1, route.visited_nodes[j + 1] + 1]
                
                # If the swap improves the route, perform the swap
                if Δ_swap < 0
                    route.visited_nodes[i], route.visited_nodes[j] = route.visited_nodes[j], route.visited_nodes[i]
                    recalculate_route!(route, inst)
                    totalΔ += Δ_swap
                    return Δ_swap
                end
            end
        end
        iterations += 1
    end
    return totalΔ
end

# Apply the reinsert first improvement heuristic to a route
function reinsert_first_improvement!(route::Route, inst::PVRPInstanceStruct, max_iterations::Int=1000)::Number
    Δ, totalΔ, iterations = -1, 0, 0

    while Δ != 0 && iterations < max_iterations
        Δ = 0
        for i in 2:(length(route.visited_nodes) - 1)
            node = route.visited_nodes[i]
            for j in 2:(length(route.visited_nodes) - 1)
                if i != j
                    # Create a temporary route with the node reinserted at a new position
                    temp_nodes = vcat(route.visited_nodes[1:(i - 1)], route.visited_nodes[(i + 1):end])
                    temp_nodes = vcat(temp_nodes[1:(j - 1)], [node], temp_nodes[j:end])
                    
                    # Calculate the length of the temporary route
                    temp_length = sum(inst.distance_matrix[temp_nodes[k] + 1, temp_nodes[k + 1] + 1] for k in 1:(length(temp_nodes) - 1))
                    current_length = sum(inst.distance_matrix[route.visited_nodes[k] + 1, route.visited_nodes[k + 1] + 1] for k in 1:(length(route.visited_nodes) - 1))
                    
                    # Calculate the change in length
                    Δ_reinsert = temp_length - current_length
                    
                    # If the reinsert improves the route, perform the reinsert
                    if Δ_reinsert < 0
                        route.visited_nodes = temp_nodes
                        recalculate_route!(route, inst)
                        totalΔ += Δ_reinsert
                        return Δ_reinsert
                    end
                end
            end
        end
        iterations += 1
    end
    return totalΔ
end

end # module LocalSearch
