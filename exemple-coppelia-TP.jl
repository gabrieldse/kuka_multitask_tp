 #connexion avec VREP
PATHCSIM="/home/gabriel/Polytech/S9/kuka_multitache/";
include("/home/gabriel/Polytech/S9/kuka_multitache/lib-robotique.jl"); 
CreateRobotKukaLwr()
include("/home/gabriel/Polytech/S9/kuka_multitache/lib-CSim.jl")

global clientID=startsimulation(simx_opmode_oneshot) # On lance une instance de connexion avec VREP

if clientID==0 println("Connexion établie")
    else println("Erreur de connexion")
end

T7=zeros(4,4);
init_pos()

global rob=CreateRobotKukaLwr();
global θinit=[0.143019, -0.109465, -0.011994, -1.1788, -0.154233, 0.93555, 0.264868];
global pA=[-0.3668, -0.0379, 0.8634];

# Robot en position initiale
setjointposition(clientID,θinit,7,0,objectname_kuka)
sleep(2)

print("pos=",getjointposition(clientID,7,0,objectname_kuka))

stopsimulation(clientID,simx_opmode_oneshot) # Arrêt de la simulation


