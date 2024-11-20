# Lib CopppeliaSim philippe.fraisse@lirmm.fr

using Libdl

global simx_opmode_oneshot_wait=65536;
global simx_opmode_buffer =393216
global simx_opmode_streaming=131072
global simx_return_ok=0;
global simx_opmode_oneshot=0;
global timeOutInMs=5000;
global commThreadCycleInMs=5;

global objectname_hoap3=["WAIST_R_LR_joint", "R_LR_R_LAA_joint", "R_LAA_R_LFE_joint" ,"R_LFE_R_KN_joint" ,"R_KN_R_AFE_joint" ,"R_AFE_R_AAA_joint", "CHEST_R_SFE_joint", "R_SFE_R_SAA_joint", "R_SAA_R_SHR_joint", "R_SHR_R_EB_joint", "WAIST_L_LR_joint", "L_LR_L_LAA_joint", "L_LAA_L_LFE_joint", "L_LFE_L_KN_joint", "L_KN_L_AFE_joint", "L_AFE_L_AAA_joint", "CHEST_L_SFE_joint", "L_SFE_L_SAA_joint", "L_SAA_L_SHR_joint", "L_SHR_L_EB_joint", "WAIST_CHEST_joint"];
global objectname_robotis=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6" ];
global objectname_kuka=[ "LBR4p_joint1", "LBR4p_joint2", "LBR4p_joint3", "LBR4p_joint4","LBR4p_joint5","LBR4p_joint6","LBR4p_joint7"];
global Bazar_robot=["MPO700_fake_joint_theta", "MPO700_fake_joint_x", "MPO700_fake_joint_y", "LBR4p_joint1_left", "LBR4p_joint2_left", "LBR4p_joint3_left", "LBR4p_joint4_left","LBR4p_joint5_left", "LBR4p_joint6_left", "LBR4p_joint7_left", "LBR4p_joint1", "LBR4p_joint2", "LBR4p_joint3", "LBR4p_joint4", "LBR4p_joint5", "LBR4p_joint6", "LBR4p_joint7" ];
global objectname_jaco=["Jaco_joint1", "Jaco_joint2", "Jaco_joint3", "Jaco_joint4", "Jaco_joint5", "Jaco_joint6" ];
global objectname_LBA=["LBA_Joint_0", "LBA_Joint_1", "LBA_Joint_2", "LBA_Joint_3", "LBA_Joint_4", "LBA_Joint_5" ];
global objectname_pioneer=["Pioneer_p3dx_leftMotor", "Pioneer_p3dx_rightMotor", "Pioneer_p3dx"];

testpath=@isdefined PATHCSIM
if testpath==true
push!(Libdl.DL_LOAD_PATH,PATHCSIM);
end

function startsimulation(simx_opmode)
clientID=ccall((:simxStart,"remoteApi"),Int32,(Ptr{String},Int32,Bool,Bool,Int32,Int32),pointer("127.0.0.1"),5555,true,true,5000,5);
simxSynchronous(clientID,true);
ccall((:simxStartSimulation,"remoteApi"),Int32,(Cint,Cint),clientID,simx_opmode);
return clientID
end

function setjointforce(clientID,force,Nbrejoints,simx_opmode,objectname)
    handle=zeros(Cint,1);
    handles=zeros(Cint,Nbrejoints);
    for i=1:Nbrejoints
        cerror=ccall((:simxGetObjectHandle,"remoteApi"),Cint,(Cint,Cstring,Ptr{Cint},Cint),clientID,objectname[i],pointer(handle),simx_opmode);
        handles[i]=handle[1];
       simxPauseCommunication(clientID,1);
        ccall((:simxSetJointForce,"remoteApi"),Cint,(Cint,Cint,Cfloat,Cint),clientID,handles[i],force[i],simx_opmode)
       simxPauseCommunication(clientID,0);
    end
return
end

function setjointposition(clientID,q,Nbrejoints,simx_opmode,objectname)
handle=zeros(Cint,1);
handles=zeros(Cint,Nbrejoints);
for i=1:Nbrejoints
    cerror=ccall((:simxGetObjectHandle,"remoteApi"),Cint,(Cint,Cstring,Ptr{Cint},Cint),clientID,objectname[i],pointer(handle),simx_opmode);
    handles[i]=handle[1];
   simxPauseCommunication(clientID,1);
    ccall((:simxSetJointTargetPosition,"remoteApi"),Cint,(Cint,Cint,Cfloat,Cint),clientID,handles[i],q[i],simx_opmode)
   simxPauseCommunication(clientID,0);
end
return
end

function setjointvelocity(clientID,qdot,Nbrejoints,simx_opmode,objectname)
handle=zeros(Cint,1);
handles=zeros(Cint,Nbrejoints);
for i=1:Nbrejoints
    cerror=ccall((:simxGetObjectHandle,"remoteApi"),Cint,(Cint,Cstring,Ptr{Cint},Cint),clientID,objectname[i],pointer(handle),simx_opmode);
    handles[i]=handle[1];
   simxPauseCommunication(clientID,1);
    ccall((:simxSetJointTargetVelocity,"remoteApi"),Cint,(Cint,Cint,Cfloat,Cint),clientID,handles[i],qdot[i],simx_opmode)
   simxPauseCommunication(clientID,0);
end
return
end

function getobjectposition(clientID,simx_opmode,objectname)
q=zeros(Cfloat,3);
qread=zeros(Cfloat,3);
handle=zeros(Cint,1);
handles=zeros(Cint,1);
cerror=ccall((:simxGetObjectHandle,"remoteApi"),Cint,(Cint,Cstring,Ptr{Cint},Cint),clientID,objectname[3],pointer(handle),simx_opmode);
handles[1]=handle[1];
ccall((:simxGetObjectPosition,"remoteApi"),Cint,(Cint,Cint,Cint,Ptr{Cfloat},Cint),clientID,handles[1],-1,pointer(q),simx_opmode)
return q
end


function getobjectorientation(clientID,simx_opmode,objectname)
q=zeros(Cfloat,3);
handle=zeros(Cint,1);
handles=zeros(Cint,1);
cerror=ccall((:simxGetObjectHandle,"remoteApi"),Cint,(Cint,Cstring,Ptr{Cint},Cint),clientID,objectname[3],pointer(handle),simx_opmode);
handles[1]=handle[1];
ccall((:simxGetObjectOrientation,"remoteApi"),Cint,(Cint,Cint,Cint,Ptr{Cfloat},Cint),clientID,handles[1],-1,pointer(q),simx_opmode)
return q
end


function getobjectvelocity(clientID,simx_opmode,objectname)
V=zeros(Cfloat,3);
q=zeros(Cfloat,3);
handle=zeros(Cint,1);
handles=zeros(Cint,1);
cerror=ccall((:simxGetObjectHandle,"remoteApi"),Cint,(Cint,Cstring,Ptr{Cint},Cint),clientID,objectname[3],pointer(handle),simx_opmode);
handles[1]=handle[1];
ccall((:simxGetObjectVelocity,"remoteApi"),Cint,(Cint,Cint,Ptr{Cfloat},Ptr{Cfloat},Cint),clientID,handles[1],pointer(V),pointer(q),simx_opmode)
return V,qcontrôle
end

function getjointposition(clientID,Nbrejoints,simx_opmode,objectname)
q=zeros(Cfloat,1);
qread=zeros(Cfloat,Nbrejoints);
handle=zeros(Cint,1);
handles=zeros(Cint,Nbrejoints);
for i=1:Nbrejoints
cerror=ccall((:simxGetObjectHandle,"remoteApi"),Cint,(Cint,Cstring,Ptr{Cint},Cint),clientID,objectname[i],pointer(handle),simx_opmode);
handles[i]=handle[1];
ccall((:simxGetJointPosition,"remoteApi"),Cint,(Cint,Cint,Ptr{Cfloat},Cint),clientID,handles[i],pointer(q),simx_opmode)
qread[i]=q[1];
end
return qread
end

function getjointforce(clientID,Nbrejoints,simx_opmode,objectname)
q=zeros(Cfloat,1);
force=zeros(Cfloat,Nbrejoints);
handle=zeros(Cint,1);
handles=zeros(Cint,Nbrejoints);
for i=1:Nbrejoints
cerror=ccall((:simxGetObjectHandle,"remoteApi"),Cint,(Cint,Cstring,Ptr{Cint},Cint),clientID,objectname[i],pointer(handle),simx_opmode);
handles[i]=handle[1];
ccall((:simxGetJointForce,"remoteApi"),Cint,(Cint,Cint,Ptr{Cfloat},Cint),clientID,handles[i],pointer(q),simx_opmode)
force[i]=q[1];
end
return force
end

#simxPauseCommunication(clientID, 1);

function simxPauseCommunication(clientID,v)
    ccall((:simxPauseCommunication,"remoteApi"),Int32,(Cint,Cint),clientID,v)
end

function stopsimulation(clientID,simx_opmode)
ccall((:simxStopSimulation,"remoteApi"),Int32,(Cint,Cint),clientID,simx_opmode)
ccall((:simxFinish,"remoteApi"),Int32,(Cint,Cint),clientID,simx_opmode)
end

function simxSynchronous(clientID,v)
    ccall((:simxSynchronous,"remoteApi"),Int32,(Cint,Bool),clientID,v)
end

# cerror=ccall((:simxGetObjectHandle,"/Users/fraisse/V-REP-JULIA/remoteApi"),Cint,(Cint,Cstring,Ptr{Cint},Cint),0,"joint3",pointer(handle),simx_opmode_oneshot_wait);

function simxSynchronousTrigger(clientID,v)
    ccall((:simxSynchronous,"remoteApi"),Int32,(Cint,Cint),clientID,v)
end

function controlrobot(q,clientID,opmode1,opmode2,N,objectname)
qread=zeros(Cfloat,N);
simxSynchronous(clientID,true);
setjointposition(clientID,q,N,opmode1,objectname);
qread=getjointposition(clientID,N,opmode2,objectname);
simxSynchronousTrigger(clientID,0);
return qread
end

function controlrobotvelocity(q,clientID,opmode1,opmode2,N,objectname)
qread=zeros(Cfloat,N);
simxSynchronous(clientID,true);
setjointvelocity(clientID,q,N,opmode1,objectname);
qread=getjointposition(clientID,N,opmode2,objectname);
simxSynchronousTrigger(clientID,0);
return qread
end

function init_pos()
    qinit=[0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    setjointposition(clientID,qinit,7,0,objectname_kuka)
    sleep(1)
    θ0=[0.143019, -0.109465, -0.011994, -1.1788, -0.154233, 0.93555, 0.264868]
    setjointposition(clientID,θ0,7,0,objectname_kuka)
    sleep(1)
    end

