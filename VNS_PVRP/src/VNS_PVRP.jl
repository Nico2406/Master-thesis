module VNS_PVRP

# Include all necessary files
include("PVRPInstance.jl")
include("PVRPSolution.jl")
include("ConstructionHeuristics.jl")
include("LocalSearch.jl")
include("Shaking.jl")
include("VNS.jl")

# Import submodules
using .PVRPInstance: Node, PVRPInstanceStruct, initialize_instance, plot_instance, fill_distance_matrix!
using .Solution: Route, PVRPSolution, VRPSolution, plot_solution, plot_solution!, validate_solution, display_solution, VNSLogbook, initialize_logbook, update_logbook!, plot_logbook
using .ConstructionHeuristics: nearest_neighbor
using .LocalSearch: local_search!
using .Shaking: shaking!, change_visit_combinations!
using .VNS: vns!, test_vns!

using Plots 
using Random

end # module VNS_PVRP
