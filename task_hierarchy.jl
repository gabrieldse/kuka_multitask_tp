# Path to resources
PATHCSIM="/home/gabriel/Polytech/S9/kuka_multitache/";
include("/home/gabriel/Polytech/S9/kuka_multitache/lib-robotique.jl"); 
include("/home/gabriel/Polytech/S9/kuka_multitache/lib-CSim.jl");

using Plots

function multi_task(θinit,g1,g2,tol_1,tol_2)
    
    """

    """

    # Init variables (J1, J2, g_1, g_2, tol_1, tol_2) (e_1, e_2)
    e_1 = e_2 = 3 # init high value of error
    θ = θinit 
    CoMactuel = CoM0
    #DEBUG
    steps = 3

    # Init vectors of plot (trajectory_θ, trajectory_z, trajectory_CoM)
    trajectory_θ = Vector{Vector{Float64}}()
    trajectory_z = Vector{Vector{Float64}}()
    trajectory_CoM = Vector{Vector{Float64}}()

    # Calculate My current theta********

    pact = MGD(θinit, rob)[1:3, 4]

    # Calculte e_1 et e_2, update les jacobienes et les crop,  check convergence
        for i = 1:steps
            println("step:",i)

            J1 = Jacobian(θ,rob,pact)
            # show(stdout,"text/plain", J1); print("\n")
            J1_crop = J1[3:3, :]
            # println("J1_crop size = ", size(J1_crop))
            # show(stdout,"text/plain", J1_crop); print("\n")
            J2 = JacobianCoM(θ,rob,CoM0)
            # println("J2 = ", size(J2))
            J2_crop = J2[1:2,:]
            # println("J2_crop = ", size(J2_crop))


            e1 = g1 - pact
            e2 = g2 - CoMactuel
            # show(stdout,"text/plain", e1)
            println(".")

            # Calculer J1 et son action u1 = j_1+ e_1
            u1 = pinv(J1_crop)*e1[3]
            # println("ut1",u1)
            # Calcular sa projection P1 = I - J_1+J_1 (combien reste)
            P1 = Matrix{Float64}(I, 7, 7) - pinv(J1_crop)*J1_crop #*** why is it 1x1
            # println("P1: ")
            # show(stdout,"text/plain", P1); 
            # Calculater u2 = (J2Pi)+*(e2-J1u1) # since I cropt the jacobian I have to crop and isolate the error on the same axis
            u2 = pinv(J2_crop*P1)*(e2[1:2,:]-J2_crop*u1)
            # println("ut2",u2)
            # Calculate u_f = u1 + u2

            uf = u1 + u2
            # println("sieze ut",size(ut))
            # update θ = θ +  u_f*dt
            θ = θ + uf
            pact = MGD(θinit, rob)[1:3, 4]
            # return trajectories, success

        end
        

    # plot graph




   println("end of function")
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

CoM_target = [0, 0.1 , CoM0[3]]
pinit = MGD(θinit,rob)
global pinit = pinit[1:3,4]

println("pinit:",pinit)

# Task hirearchy

Z_target=[pinit[1], pinit[2], 0.5];
println("Z_target:", Z_target)
multi_task(θinit,Z_target,CoM_target,1e-3,1e-3)