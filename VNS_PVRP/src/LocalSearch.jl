module LocalSearch

using Main.Instance: PVRPInstance
using Main.Solution: VRPSolution, Route, recalculate_route!

export local_search!, two_opt_first_improvement!, two_opt_first_improvement_1iteration!

function local_search!(sol::VRPSolution, inst::PVRPInstance, method::String, max_iterations::Int)::Number
    Δ = 0
    for route in sol.routes
        if route.changed
            Δ += local_search!(route, inst, method, max_iterations)
        end
    end
    sol.total_length += Δ
    sol.total_duration += Δ
    return Δ
end

function local_search!(route::Route, inst::PVRPInstance, method::String, max_iterations::Int)::Number
    Δ = 0
    if method == "2opt-first"
        Δ += two_opt_first_improvement!(route, inst, max_iterations)
    else
        throw(ArgumentError("local_search! in Route.jl: method '$method' is not implemented"))
    end
    route.changed = false
    return Δ
end

function two_opt_first_improvement!(route::Route, inst::PVRPInstance, max_iterations::Int=1000)::Number
    Δ, totalΔ, iterations = -1, 0, 0
    while Δ != 0 && iterations < max_iterations
        Δ = two_opt_first_improvement_1iteration!(route, inst)
        totalΔ += Δ
        iterations += 1
    end
    return totalΔ
end

function two_opt_first_improvement_1iteration!(route::Route, inst::PVRPInstance)::Number
    routelen = length(route.visited_nodes)
    for i = 2:(routelen - 2)
        for j = (i + 1):(routelen - 1)
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

end # module
