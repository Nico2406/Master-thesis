module Instance

using Plots

export Node, PVRPInstance, read_instance, fill_distance_matrix!, plot_instance, get_fitting_layout, initialize_instance

mutable struct Node
    id::Int64
    x::Float64
    y::Float64
    demand::Int64
    ready_time::Int64
    due_time::Int64
    service_time::Int64
    frequency::Int64
    number_of_combinations::Int64
    visitcombinations::Vector{Vector{Bool}}
    initialvisitcombination::Vector{Bool}
    initialvisitcombinationint::Int64

    function Node(; id::Int64, x::Float64, y::Float64, demand::Int64, ready_time::Int64, due_time::Int64, service_time::Int64, frequency::Int64, number_of_combinations::Int64, visitcombinations::Vector{Vector{Bool}}, initialvisitcombination::Vector{Bool}, initialvisitcombinationint::Int64)
        new(id, x, y, demand, ready_time, due_time, service_time, frequency, number_of_combinations, visitcombinations, initialvisitcombination, initialvisitcombinationint)
    end
end

mutable struct PVRPInstance
    problemtype::Int
    numberofvehicles::Int
    numberofcustomers::Int
    numberofdays::Int
    maximumrouteduration::Int
    vehicleload::Int
    nodes::Vector{Node}
    distance_matrix::Matrix{Float64}

    function PVRPInstance(
        problemtype::Int, numberofvehicles::Int, numberofcustomers::Int,
        numberofdays::Int, maximumrouteduration::Int, vehicleload::Int, nodes::Vector{Node}
    )
        matrix = fill(Inf, numberofcustomers + 1, numberofcustomers + 1)::Matrix{Float64}
        new(problemtype, numberofvehicles, numberofcustomers, numberofdays, maximumrouteduration, vehicleload, nodes, matrix)
    end
end

function convert_int_binary(v::Int, digits::Int)::Vector{Bool}
    return parse.(Bool, split(bitstring(v)[(end - digits + 1):end], ""))
end

function convert_binary_int(bits::Vector{Bool})::Int
    return Int(reduce((x, y) -> (x << 1) | UInt64(y), bits, init = 0))
end

function fill_distance_matrix!(instance::PVRPInstance)
    num_nodes = length(instance.nodes)
    instance.distance_matrix .= Inf  

    for i in 1:num_nodes  # include depot at index 1
        for j in 1:num_nodes
            if i == j
                instance.distance_matrix[i, j] = 0.0
            else
                x1, y1 = (i == 1) ? (instance.nodes[1].x, instance.nodes[1].y) : (instance.nodes[i].x, instance.nodes[i].y)
                x2, y2 = (j == 1) ? (instance.nodes[1].x, instance.nodes[1].y) : (instance.nodes[j].x, instance.nodes[j].y)
                dist = sqrt((x2 - x1)^2 + (y2 - y1)^2)::Float64
                instance.distance_matrix[i, j] = dist
            end
        end
    end
end

function calc_distance_matrix(x::Vector{Float64}, y::Vector{Float64}, rounding_factor = 5)::Matrix{Float64}
    n = length(x)
    # distance matrix calculation
    return [round(hypot(x[i] - x[j], y[i] - y[j]), digits = rounding_factor) for j = 1:n, i = 1:n]
end

function read_instance(filepath::String)::PVRPInstance
    # Read file lines
    lines = readlines(filepath)

    # Parse general problem info
    problem_info = parse.(Int, filter(!isempty, split(strip(lines[1]))))
    problemtype, numberofvehicles, numberofcustomers, numberofdays = problem_info

    # Parse route info for max route duration and vehicle load
    route_info = [parse(Int, x) for x in split(strip(lines[2])) if !isempty(x)]
    maximumrouteduration, vehicleload = route_info
    if maximumrouteduration == 0
        maximumrouteduration = typemax(Int)
    end

    # Prepare nodes vector, starting with the depot as the first node
    nodes = Vector{Node}()

    # Parse the depot line
    depot_line_index = 2 + numberofdays
    depot_line = lines[depot_line_index]
    depot_info = [parse(Float64, x) for x in split(strip(depot_line)) if !isempty(x)]
    depot_id = Int(depot_info[1])
    depot_x, depot_y = depot_info[2:3]
    depot_node = Node(
        id=depot_id,
        x=depot_x,
        y=depot_y,
        demand=0,
        ready_time=0,
        due_time=0,
        service_time=0,
        frequency=0,
        number_of_combinations=0,
        visitcombinations=Vector{Vector{Bool}}(),
        initialvisitcombination=Vector{Bool}(undef, numberofdays),
        initialvisitcombinationint=0
    )
    push!(nodes, depot_node)

    # Parse customer lines and add them to the nodes vector
    for i in (depot_line_index + 1):length(lines)
        if isempty(strip(lines[i]))
            continue
        end
        customer_info = [tryparse(Float64, x) for x in filter(!isempty, split(strip(lines[i])))]
        if length(customer_info) >= 7
            id = Int(customer_info[1])
            xcoord, ycoord = customer_info[2:3]
            serviceduration = Int(customer_info[4])  # Ensure service_time is Int64
            demand = Int(customer_info[5])  # Ensure demand is Int64
            frequency = Int(customer_info[6])  # Ensure frequency is Int64
            numbervisitcombinations = Int(customer_info[7])  # Ensure number_of_combinations is Int64
            visitcombinations = [convert_int_binary(Int(x), numberofdays) for x in customer_info[8:end]]
            initialvisitcombination = visitcombinations[1]  # Set initial combination
            initialvisitcombinationint = convert_binary_int(initialvisitcombination)
            customer_node = Node(
                id=id,
                x=xcoord,
                y=ycoord,
                demand=demand,
                ready_time=0,
                due_time=0,
                service_time=serviceduration,
                frequency=frequency,
                number_of_combinations=numbervisitcombinations,
                visitcombinations=visitcombinations,
                initialvisitcombination=initialvisitcombination,
                initialvisitcombinationint=initialvisitcombinationint
            )
            push!(nodes, customer_node)
        end
    end

    # Create the PVRPInstance with nodes and distance matrix
    instance = PVRPInstance(
        problemtype, numberofvehicles, numberofcustomers, numberofdays,
        maximumrouteduration, vehicleload, nodes
    )
    
    # Fill distance matrix based on node positions
    fill_distance_matrix!(instance)

    return instance
end

function plot_instance(inst::PVRPInstance; plotsize = (2000, 2000), show_legend = true, tickfontsize = 15)
    x_coords = [node.x for node in inst.nodes]
    y_coords = [node.y for node in inst.nodes]

    x_diff = maximum(x_coords) - minimum(x_coords)
    x_buff = x_diff / 20
    xlimits = (minimum(x_coords) - x_buff, maximum(x_coords) + x_buff)
    y_diff = maximum(y_coords) - minimum(y_coords)
    y_buff = y_diff / 20
    ylimits = (minimum(y_coords) - y_buff, maximum(y_coords) + y_buff)

    p = plot(
        x_coords[2:end],
        y_coords[2:end],
        seriestype = :scatter,
        markershape = :circle,
        markercolor = :slategrey,
        markerstrokecolor = :slategrey,
        markersize = 5,
        aspect_ratio = :equal,
        xlimits = xlimits,
        ylimits = ylimits,
        size = plotsize,
        label = "customers",
        legend = show_legend,
        tickfontsize = tickfontsize,
    )

    plot!(p, x_coords[1:1], y_coords[1:1], seriestype = :scatter, markershape = :rect, markercolor = :black, markerstrokecolor = :black, markersize = 7, label = "depot")

    return p
end

function get_fitting_layout(nr_periods::Int)
    nr_rows = ceil(Int, sqrt(nr_periods))
    nr_columns = ceil(Int, nr_periods / nr_rows)
    return @layout [grid(nr_rows, nr_columns)]
end

function initialize_instance(file_path::String)::PVRPInstance
    instance = read_instance(file_path)
    fill_distance_matrix!(instance)
    return instance
end

end # module Instance