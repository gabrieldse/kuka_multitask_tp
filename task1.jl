using Plots
using Printf

# Path to resources
PATHCSIM="/home/gabriel/Polytech/S9/kuka_multitache/";
include("/home/gabriel/Polytech/S9/kuka_multitache/lib-robotique.jl"); 
include("/home/gabriel/Polytech/S9/kuka_multitache/lib-CSim.jl");

function cininv(θinit,rob,x_target,step,do_plot=1,max_iterations=200,tol=1e-3,dt=0.05)
    println("[Task End Effector along Z] Starting task hierarchy...")
    formatted_x_target = join([@sprintf("%.2f", x) for x in x_target], ", ")
    println("[Task End Effector along Z] Goal position end effector = ", formatted_x_target )


    """
    Perform inverse kinematics to move the robot's end-effector to `x_target`.

    Returns 3x1 vector of the trajectory_xyz of traslations.
    Returns Nx1 vector of the trajectory_θ of motor angles.

    Parameters:
        θd - angle velocity
        [e_x;zeros(3)] = change in position and orientation
    """

    # Init variables
    trajectory_xyz = Vector{Vector{Float64}}()
    trajectory_θ = Vector{Vector{Float64}}()
    θcurrent = θinit
    e_x = [sum(rob.r);sum(rob.r);sum(rob.r)]
    i = 1
    println("[Task End Effector along Z] Iterating...")
    while (norm(e_x) > tol)
        
        # 1 - Get current position via MGD
        Tcurrent = MGD(θcurrent,rob)
        x_current = Tcurrent[1:3,4]
        push!(trajectory_θ,θcurrent)

        # 2 - Calculate error (Δx = velocity)
        e_x = x_target - x_current
        J = Jacobian(θinit,rob,x_current)
        J_trans = J[1:3, :]

        # 3 - Compute Δθ to correct error
        θd = pinv(J_trans) * e_x./step

        # 4 - Update current angle
        θcurrent += θd
        setjointposition(clientID,θcurrent,7,0,objectname_kuka)
        push!(trajectory_xyz,x_current)
        sleep(dt)

        i = i + 1
        if (i == max_iterations)
            println("[Task End Effector along Z] Done in ",i," steps. Error norm = ",@sprintf("%.3f",norm(e_x))," [m]")

            if (do_plot==1)
                # Plot values
                z_values = [element[3] for element in trajectory_xyz]
                pθ = hcat(trajectory_θ...).* 180 / pi
                pz = plot(z_values, label="z", xlabel="Iteration", ylabel="Position", title="Z Component")
                pθ2 =  plot(pθ', label=["θ₁" "θ₂" "θ₃" "θ₄" "θ₅" "θ₆" "θ₇"], xlabel="Iteration", ylabel="Motors angles", title="Motors angles")
                p = plot(pz, pθ2, layout=(2, 1))
                display(p)
            end

            return trajectory_xyz, trajectory_θ
            break
        end

    end

    if (do_plot==1)
        # Plot values
        z_values = [element[3] for element in trajectory_xyz]
        pθ = hcat(trajectory_θ...)
        pz = plot(z_values, label="z", xlabel="Iteration", ylabel="Position", title="Z Component")
        pθ2 =  plot(pθ', label=["θ₁" "θ₂" "θ₃" "θ₄" "θ₅" "θ₆" "θ₇"], xlabel="Iteration", ylabel="Motors angles", title="Motors angles")
        p = plot(pz, pθ2, layout=(2, 1))
        display(p)
    end

    formated_z_values = join([@sprintf("%.2f", z) for z in z_values[end]], ", ")
    println("[Task End Effector along Z] Final end effector position  = ", formated_z_values)
    
    formatted_trajector_θ = join([@sprintf("%.2f", x) for x in trajectory_θ[end].* 180/pi], ", ")
    println("[Task End Effector along Z] Final θ angles = ", formatted_trajector_θ)

    return trajectory_xyz, trajectory_θ
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
global pA=[-0.3668, -0.0379, 0.8634];
x_target = [pA[1], pA[2], 0.5]

# Calculate Inverse kinematics
trajectory_xyz, trajectory_θ = cininv(θinit,rob,x_target,50)