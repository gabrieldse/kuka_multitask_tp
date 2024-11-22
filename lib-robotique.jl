using LinearAlgebra

mutable struct DH
    α::Array{Float64,1}
    d::Array{Float64,1}
    r::Array{Float64,1}
    m::Array{Float64,1}
    c::Array{Float64,2}
end


function CreateRobotKukaLwr()
 α=[0.0,pi/2,-pi/2,-pi/2,pi/2,pi/2,-pi/2];
 d=zeros(7);
 r=[0.3105,0.0,0.4,0.0,0.39,0.0,0.078]
 θ=zeros(7);
 m=[2.7,2.7,2.7,2.7,2.7,2.7,0.3];
 c=zeros(4,7)
 c[2,:]=[-8.70e-3, 8.7e-3, 8.7e-3, -8.7e-3,-8.2e-3, -7.6e-3, 0.0];
 c[3,:]=[-1.461e-2,1.461e-2,-1.461e-2, 1.461e-2,-3.48e-2, 1.363e-3, 0.0];
 c[4,:]=[1.0,1.0,1.0,1.0,1.0,1.0,1.0];
 robot_kuka=DH(α,d,r,m,c)
 return robot_kuka
end

function MGD(θ,robot)
    # Notation Tp=Matrix(I,4,4) - creates a Indentity matrix of 4x4. I is the identity in julia
    
    # Equations on the Jupter notebook
    N=length(robot.α);
    T=zeros(4,4);
    Tp=Matrix(I,4,4);
     for i=1:N
        cθ=cos(θ[i]); sθ=sin(θ[i]); d=robot.d[i]; sα=sin(robot.α[i]); cα=cos(robot.α[i]); r=robot.r[i];
        T=Tp*[ cθ     -sθ    0.0    d;
              cα*sθ  cα*cθ   -sα   -r*sα;
              sα*sθ  sα*cθ    cα    r*cα;
               0.0    0.0     0.0    1.0  ];
        Tp=T;
    end
    return T
end

function Jacobian(θ,robot,p0)
    # It output is a 6xN
    # p_dot = J(θ_dot) - The jacobian relates the joint velocities with the end-effector velocity
    # p0 - position of the end-effector
    # pi = T[1:3,4] = position of the joint i
    # zi = T[1:3,3] = orientation of the joint i
    # l - vector that points to the p0 (end effector) from the joint i

    # Equations on the Jupter notebook
    N=length(robot.α);
    T=zeros(4,4); l=zeros(3,N); z=zeros(3,N);
    Tp=Matrix(I,4,4);
    J=zeros(6,N);
    for i=1:N
        cθ=cos(θ[i]); sθ=sin(θ[i]); d=robot.d[i]; sα=sin(robot.α[i]); cα=cos(robot.α[i]); r=robot.r[i];
        T=  Tp*  [ cθ     -sθ    0.0    d;
              cα*sθ  cα*cθ   -sα   -r*sα;
              sα*sθ  sα*cθ    cα    r*cα;
               0.0    0.0     0.0    1.0 ];
       l[:,i]=p0[:]-T[1:3,4];
       z[:,i]=T[1:3,3];
       J[1:3,i]=cross(z[:,i],l[:,i]);
       J[4:6,i]=z[:,i];
       Tp=T;
    end
    return J
end

function CoM(θ,robot)
    # T transforms the local coordinates of the i-th link's center of mass into the global coordinate frame.
    # T * robot.c[:, i]: Transforms the center of mass from the local frame to the global frame.
    # Gets the current contribution ((robot.m[i]/M).)
    """ return 3x1 CoM position on the robot coordinate frame """

        N=length(robot.α);
        M=sum(robot.m[1:N]);
        T=zeros(4,4);
        CoM=zeros(4,1);CoMp=zeros(4,1);
        Tp=Matrix(I,4,4);
         for i=1:N
            cθ=cos(θ[i]); sθ=sin(θ[i]); d=robot.d[i]; sα=sin(robot.α[i]); cα=cos(robot.α[i]); r=robot.r[i];
            T=Tp*[ cθ     -sθ    0.0    d;
                  cα*sθ  cα*cθ   -sα   -r*sα;
                  sα*sθ  sα*cθ    cα    r*cα;
                  0.0    0.0     0.0    1.0  ];
            CoM[:]=(robot.m[i]/M).*T*robot.c[:,i]+CoMp; 
            CoMp=CoM;
            Tp=T;
        end
        return CoM
end

function JacobianCoM(θ,robot,CoM0)
    # output is a 3xN matrix with the jacobian of each segment CoM jacobian as vectors. From 1 to N.
    N=length(robot.α);
    M=sum(robot.m[1:N]);
    CoM=zeros(4,1);CoMp=zeros(4,1);
    T=zeros(4,4); l=zeros(3,N); z=zeros(3,N);
    Tp=Matrix(I,4,4);
    CoMo=zeros(4,1);
    Jcom=zeros(3,N);
    for i=1:N
        cθ=cos(θ[i]); sθ=sin(θ[i]); d=robot.d[i]; sα=sin(robot.α[i]); cα=cos(robot.α[i]); r=robot.r[i];
        T= Tp*[ cθ     -sθ    0.0    d;
            cα*sθ  cα*cθ   -sα   -r*sα;
            sα*sθ  sα*cθ    cα    r*cα;
            0.0    0.0     0.0    1.0 ];
    CoMo[:]=(robot.m[i]/M).*T*robot.c[:,i]+CoMp;
    CoMp=CoMo;
    l[:,i]=CoM0[1:3]-CoMo[1:3];
    z[:,i]=T[1:3,3];
    Jcom[1:3,i]=cross(z[:,i],l[:,i]);
    Tp=T;
    end
    return Jcom
end


#-----------------------
#θ=[0.143019, -0.109465, -0.011994, -1.1788, -0.154233, 0.93555, 0.264868];
#-----------------------


function RTL2R(ϕ,θ,ψ)
    A=[cos(θ)*cos(ϕ)  -sin(ϕ)*cos(ψ) + cos(ϕ)*sin(θ)*sin(ψ)   sin(ψ)*sin(ϕ) + cos(ϕ)*sin(θ)*cos(ψ)
       cos(θ)*sin(ϕ)   cos(ϕ)*cos(ψ) + sin(θ)*sin(ψ)*sin(ϕ)  -cos(ϕ)*sin(ψ) + sin(θ)*sin(ϕ)*cos(ψ)
       -sin(θ)                          cos(θ)*sin(ψ)                          cos(θ)*cos(ψ) ];
return A
end

function R2RTL(R)
    θ=-asin(R[3,1]);
    ϕ=atan(R[2,1],R[1,1]);
    ψ=atan(R[3,2],R[3,3]);
    return ϕ,θ,ψ
end

function B0(ϕ,θ,ψ)
 R=[0  -sin(ϕ)  cos(θ)*cos(ϕ)
 0   cos(ϕ)  cos(θ)*sin(ϕ)
 1        0        -sin(θ)];
 return R
end
