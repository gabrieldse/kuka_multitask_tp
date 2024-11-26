# Path to resources
PATHCSIM="/home/gabriel/Polytech/S9/kuka_multitache/";
include("/home/gabriel/Polytech/S9/kuka_multitache/lib-robotique.jl"); 
include("/home/gabriel/Polytech/S9/kuka_multitache/lib-CSim.jl");

using Plots

function cininCoM(θinit,rob,CoMtarget,step,do_plot=1,max_iterations=200,tol=1e-3,dt=0.05)
    """
    Perform inverse kinematics to move the robot's center of mass to the `CoMtarget`.

    Returns 3x1 vector of the trajectory_CoM of traslations.
    Returns Nx1 vector of the trajectory_θ of motor angles.

    Parameters:
        θd - angle velocity
    """

    # Init values
    trajectory_CoM = Vector{Vector{Float64}}()
    trajectory_θ = Vector{Vector{Float64}}()
    θcurrent = θinit
    e_CoM = [sum(rob.r);sum(rob.r);sum(rob.r)]
    println(e_CoM)
    i = 1

    println("Initial CoM = ", CoM(θcurrent,rob))

    while (norm(e_CoM) > tol)
        # 1 - Get current position via MGD
        CoMcurrent = CoM(θcurrent,rob)
        CoMcurrent = CoMcurrent[1:3]
        println("CoMcurrent:",CoMcurrent)
        println("CoMtarget:",CoMtarget)
        println("e_CoM:",e_CoM)
        
        # CoMcurrent = CoMcurrent[1:3,4]
        push!(trajectory_θ,θcurrent)

        # 2 - Caclulate error (Δx = velocity)
        e_CoM = CoMtarget - CoMcurrent
        println("Iteration $i. Error: ", e_CoM./100,"%")
        J = JacobianCoM(θinit,rob,CoMcurrent)
        J_trans = J[1:3, :]

        # 3 - Compute Δθ to correct error
        θd = pinv(J_trans) * e_CoM./step #CoMcurrent[3] -
        # println("Mudanca de angul:",θd)

        # 4 - Update current angle
        θcurrent += θd
        setjointposition(clientID,θcurrent,7,0,objectname_kuka)
        push!(trajectory_CoM,CoMcurrent)
        sleep(dt)

        i = i + 1
        if (i == max_iterations)
            println("Too many cycles")

            if (do_plot == 1)
                CoM_value_x = [element[1] for element in trajectory_CoM]
                CoM_value_y = [element[2] for element in trajectory_CoM]
                CoM_value_z = [element[3] for element in trajectory_CoM]
            
                # Plot CoM values for x and y together
                pCoM_xy = plot(CoM_value_x, label="CoMₓ", xlabel="Iteration", ylabel="Position", title="CoM Components (X & Y)")
                plot!(pCoM_xy, CoM_value_y, label="CoMᵧ")
            
                # Plot motor θ values values 
                pθ = hcat(trajectory_θ...)
                pθ2 =  plot(pθ', label=["θ₁" "θ₂" "θ₃" "θ₄" "θ₅" "θ₆" "θ₇"], xlabel="Iteration", ylabel="Motors angles", title="Motors angles")
            
                # Arrange subplots
                p = plot(pCoM_xy, pθ2, layout=(2, 1))
                display(p)
            end
            
            return trajectory_CoM, trajectory_θ
            break
        end
    end

    if (do_plot==1)
        # Plot values
        z_values = [element[3] for element in trajectory_CoM]
        pθ = hcat(trajectory_θ...)
        pz = plot(z_values, label="z", xlabel="Iteration", ylabel="Position", title="Z Component")
        pθ2 =  plot(pθ', label=["θ₁" "θ₂" "θ₃" "θ₄" "θ₅" "θ₆" "θ₇"], xlabel="Iteration", ylabel="Motors angles", title="Motors angles")
        p = plot(pz, pθ2, layout=(2, 1))
        display(p)
    end

    return trajectory_CoM, trajectory_θ
end

# Start of the simulation
global clientID=startsimulation(simx_opmode_oneshot) # On lance une instance de connexion avec VREP
if clientID==0 println("Connected")
    else println("Connection error")
end

# Main logic
init_pos()
sleep(2)

global rob=CreateRobotKukaLwr();
global θinit=[0.143019, -0.109465, -0.011994, -1.1788, -0.154233, 0.93555, 0.264868];
global CoM0=[-0.0844, -0.01223, 0.6370];

CoMtarget = [0, 0.1 , CoM0[3]]

# Calculate Inverse kinematics
trajectory_CoM, trajectory_θ = cininCoM(θinit,rob,CoMtarget,50)