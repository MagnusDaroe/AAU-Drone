#!/usr/bin/env python3
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver, AcadosModel
import casadi as ca
import yaml
import numpy as np
import os
import scipy



class  NMPC():

    def __init__(self,robot_name):
        with open('src/trajectory_gen/config/dnmpc_params.yaml') as file:
            yamlfile = yaml.safe_load(file)
        self.__QP_solver = yamlfile['QP_solver']
        self.frequency=yamlfile['Prediction_Frequency']
        self.__prediction_length = yamlfile['Prediction_Length']
        self.update_frequency=yamlfile['Update_Frequency']
        self.numberofobs=yamlfile['Number_obstacles']
        self.numberofobs=yamlfile['Number_obstacles']
        self.__numberofobs=yamlfile['Number_obstacles']
        self.__mass=yamlfile['mass']
        self._robot_radius=yamlfile['robot_radius']
        self.robot_name=robot_name
        self.__robot_model=self.__define_model(robot_name,self.numberofobs)

        
        self.__Tf = self.__prediction_length
        self.__N=int(self.__prediction_length*self.frequency)
        self.N=self.__N 

        self.__model=self.__robot_model # Model
        self.__ocp = AcadosOcp()
        self.__ocp.model = self.__model #Define Model

        self.__n_params = self.__model.p.size()[0]



        self.prediction_length=self.__prediction_length
        self.__ocp.dims.np = self.__n_params
        self.__ocp.parameter_values = np.zeros(self.__n_params)
        self.__ocp.dims.N = self.__N

        # Vehicle parameters
        self.__nlp_solver_type=yamlfile['nlp_solver_type']
        self.__Q_matrix=yamlfile['Q_matrix'] 
        self.__R_matrix=yamlfile['R_matrix']
        self.__print_status=yamlfile['print_acados_status']
        self.__nlp_solver_max_iter=yamlfile['nlp_solver_max_iter']
        self.__integrator_type=yamlfile['integrator_type']
        
        
        self.__L2_Norm=yamlfile['L2_Norm']
        self.__L1_Norm=yamlfile['L1_Norm']
    

        self.__thrust_max= (yamlfile['thrust_max'])
        self.__thrust_min= (yamlfile['thrust_min'])

        self.__roll_max= np.deg2rad(yamlfile['roll_max'])
        self.__roll_min= np.deg2rad(yamlfile['roll_min'])

        self.__pitch_max= np.deg2rad(yamlfile['pitch_max'])
        self.__pitch_min= np.deg2rad(yamlfile['pitch_min'])

        
        

        self.__linear_cost_function()
        self.__constraints()
        self.__solver_compiler()
    
    def __generate_obstacle_params(self,number_obs):
        obs=[]
        for i in range(number_obs):
            obs_temp=np.array([ca.SX.sym('x'+str(i)+'_obs'),ca.SX.sym('y'+str(i)+'_obs'),ca.SX.sym('r'+str(i)+'_obs')]).reshape(-1,1)            
            obs.append(obs_temp)
        print("hehehehe",obs)
        return ca.vertcat(*obs)
    
    def __define_model(self,robot_name,number_obs):
        model=AcadosModel()
        current_directory=os.getcwd()


        model = AcadosModel()
        constraint = ca.types.SimpleNamespace()
        # control inputs
        roll = ca.SX.sym('v')
        pitch = ca.SX.sym('th_d')
        thrust = ca.SX.sym('T')
        controls = ca.vertcat(thrust, roll, pitch)
   
        # model states
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        z = ca.SX.sym('z')


        if number_obs>0:
            obs=self.__generate_obstacle_params(number_obs)
            paremeters  = ca.vertcat(
                    obs
                    )   
        else:
            paremeters  = ca.vertcat()

        states = ca.vertcat(x, 
                            y, 
                            z)
        
        m=self.__mass
        kin_eq = [  (thrust/m) * roll*0.1,
                    (thrust/m) * pitch*0.1,
                    ((thrust/m) - 9.81)*0.1
                ]
        f = ca.Function('f', [states,paremeters, controls], [ca.vcat(kin_eq)], ['state','paremeters', 'control_input'], ['kin_eq'])
        f_impl = ca.SX.sym('x_dot', len(kin_eq)) - f(states,paremeters, controls)

        model.f_expl_expr = f(states,paremeters, controls)
        model.f_impl_expr = f_impl
        model.x = states
       
        model.u = controls
        model.p=paremeters
        model.name = robot_name
    


        p = ca.vertcat([])
        model.p=p

        model.name = 'drone'
        

        return model
    
    def __linear_cost_function(self):
        
        # cost type
        self.__nx = self.__model.x.size()[0] 
        self.__nx = self.__nx #Number of states
        self.__nu = self.__model.u.size()[0]
        self.__ny = self.__nx + self.__nu
        self.__ny_e = self.__nx
        ### COST FUNCTION ##
        unscale=self.__N/self.__Tf
        self.__Q =  np.diag(self.__Q_matrix) # [x,y,x_d,y_d,th,th_d]



        self.__R = np.diag(self.__R_matrix)
        self.__ocp.cost.cost_type = 'LINEAR_LS'
        self.__ocp.cost.cost_type_e = 'LINEAR_LS'

        self.__ocp.cost.W = scipy.linalg.block_diag(self.__Q, self.__R)
        self.__ocp.cost.W_0 = scipy.linalg.block_diag(self.__Q, self.__R)
        print("here")
        print(self.__ocp.cost.W_0)
        print("w")
        print(self.__ocp.cost.W)

        self.__ocp.cost.W_e=  np.diag(self.__Q_matrix)# [x,y,x_d,y_d,th,th_d]
        self.__ocp.cost.Vx = np.zeros((self.__ny, self.__nx))
        self.__ocp.cost.Vx[:self.__nx, :self.__nx] = np.eye(self.__nx)
        self.__ocp.cost.Vx_0 = np.zeros((self.__ny, self.__nx))
        self.__ocp.cost.Vx_0[:self.__nx, :self.__nx] = np.eye(self.__nx)
        self.__ocp.cost.Vu = np.zeros((self.__ny, self.__nu))
        self.__ocp.cost.Vu[-self.__nu:, -self.__nu:] = np.eye(self.__nu)

        self.__ocp.cost.Vu_0 = np.zeros((self.__ny, self.__nu))
        self.__ocp.cost.Vu_0[-self.__nu:, -self.__nu:] = np.eye(self.__nu)
        self.__ocp.cost.Vx_e = np.eye(self.__nx)

        self.__ocp.cost.yref = np.ones((self.__ny,))
        self.__ocp.cost.yref_e = np.ones((self.__ny_e,))
        self.__ocp.constraints.x0 = np.ones((self.__nx,))
    
    def __constraints(self):
    

        self.__ns_e=0
        self.__ns_0=0
        self.__ns_i=0
        x_alg=self.__model.x[0]
        y_alg=self.__model.x[1]
        z_alg=self.__model.x[2]




        self.__ocp.constraints.lbu = np.array([self.__thrust_min,self.__roll_min,self.__pitch_min])
        self.__ocp.constraints.ubu = np.array([self.__thrust_max,self.__roll_max,self.__pitch_max])
        self.__ocp.constraints.idxbu = np.array([0,1,2])
        self.__ocp.constraints.lsbu = np.zeros(3)
        self.__ocp.constraints.usbu = np.zeros(3)
        self.__ocp.constraints.idxsbu = np.array([0,1,2])
        self.__ns_i+=3
        self.__ns_0+=3




        if False: 
            self.__avoidance_n=self.numberofobs
            con_h =[]
            for i in range(4, (self.__numberofobs)*3+4, 3):
                con_h.append((x_alg - self.__model.p[i+0])**2 + (y_alg - self.__model.p[i+1])**2 - (self.__model.p[i+2] + 0.1)**2)



            
            con_h_vcat=ca.vcat(con_h)


            self.__ocp.model.con_h_expr_0 =con_h_vcat
            self.__ocp.constraints.lh_0 =np.array([0]*self.__avoidance_n+ [lb_com]*self.__com)
            self.__ocp.constraints.uh_0 =np.array([100]*self.__avoidance_n+ [ub_com]*self.__com)
            self.__ocp.constraints.lsh_0 = np.zeros(len(con_h))             # Lower bounds on slacks corresponding to soft lower bounds for nonlinear constraints
            self.__ocp.constraints.ush_0 = np.zeros(len(con_h))             # Lower bounds on slacks corresponding to soft upper bounds for nonlinear constraints
            self.__ocp.constraints.idxsh_0 = np.array(range(len(con_h)))    # Jsh
            self.__ns_0+=len(con_h)
            
            self.__ocp.model.con_h_expr =con_h_vcat
            self.__ocp.constraints.lh =np.array([self.__ed_min] + [0]*self.__avoidance_n+ [lb_com]*self.__com)
            self.__ocp.constraints.uh  =np.array([self.__ed_max] + [100]*self.__avoidance_n+ [ub_com]*self.__com)
            self.__ocp.constraints.lsh = np.zeros(len(con_h))             # Lower bounds on slacks corresponding to soft lower bounds for nonlinear constraints
            self.__ocp.constraints.ush = np.zeros(len(con_h))             # Lower bounds on slacks corresponding to soft upper bounds for nonlinear constraints
            self.__ocp.constraints.idxsh = np.array(range(len(con_h)))    # Jsh
            self.__ns_i+=len(con_h)

            self.__ocp.model.con_h_expr_e =con_h_vcat
            self.__ocp.constraints.lh_e =np.array([self.__ed_min] + [0]*self.__avoidance_n+ [lb_com]*self.__com)
            self.__ocp.constraints.uh_e =np.array([self.__ed_max] + [100]*self.__avoidance_n+ [ub_com]*self.__com)
            self.__ocp.constraints.lsh_e = np.zeros(len(con_h))             # Lower bounds on slacks corresponding to soft lower bounds for nonlinear constraints
            self.__ocp.constraints.ush_e = np.zeros(len(con_h))             # Lower bounds on slacks corresponding to soft upper bounds for nonlinear constraints
            self.__ocp.constraints.idxsh_e = np.array(range(len(con_h)))    # Jsh
            self.__ns_e+=len(con_h)
        


            
        L2penalty=self.__L2_Norm
        L1pentaly=self.__L1_Norm
        
        
        ns = self.__ns_i
        self.__ocp.cost.zl = L1pentaly  * np.ones((ns,))    # gradient wrt lower slack at intermediate shooting nodes (1 to N-1)
        self.__ocp.cost.Zl = L2penalty  * np.ones((ns,))    # diagonal of Hessian wrt lower slack at intermediate shooting nodes (1 to N-1)
        self.__ocp.cost.zu = L1pentaly  * np.ones((ns,))    
        self.__ocp.cost.Zu = L2penalty  * np.ones((ns,))  



        ns_e =self.__ns_e
        self.__ocp.cost.zl_e = L1pentaly * np.ones((ns_e,))    # gradient wrt lower slack at intermediate shooting nodes (1 to N-1)
        self.__ocp.cost.Zl_e = L2penalty * np.ones((ns_e,))    # diagonal of Hessian wrt lower slack at intermediate shooting nodes (1 to N-1)
        self.__ocp.cost.zu_e = L1pentaly * np.ones((ns_e,))    
        self.__ocp.cost.Zu_e = L2penalty * np.ones((ns_e,)) 

        ns_0 =self.__ns_0
        self.__ocp.cost.zl_0 = L1pentaly * np.ones((ns_0,))    # gradient wrt lower slack at intermediate shooting nodes (1 to N-1)
        self.__ocp.cost.Zl_0 = L2penalty * np.ones((ns_0,))    # diagonal of Hessian wrt lower slack at intermediate shooting nodes (1 to N-1)
        self.__ocp.cost.zu_0 = L1pentaly * np.ones((ns_0,))    
        self.__ocp.cost.Zu_0 = L2penalty * np.ones((ns_0,)) 

    def __solver_compiler(self):
        x_ref = np.zeros(self.__nx)
        u_ref = np.zeros(self.__nu)
        # initial state
        self.__ocp.constraints.x0 = x_ref
        self.__ocp.cost.yref = np.concatenate((x_ref, u_ref))
        self.__ocp.cost.yref_e = x_ref
        self.__ocp.solver_options.levenberg_marquardt = 1.0
        #self.__oc.solver_options.reg_epsilon='CONVEXIFY'
        self.__ocp.solver_options.qp_solver = self.__QP_solver
        self.__ocp.solver_options.nlp_solver_type = self.__nlp_solver_type
        self.__ocp.solver_options.tf = self.__Tf
        self.__ocp.solver_options.integrator_type=self.__integrator_type
        self.__ocp.solver_options.nlp_solver_max_iter=self.__nlp_solver_max_iter
        self.__ocp.solver_options.qp_solver_warm_start=2

        json_file = os.path.join('_acados_ocp.json')
        
        self.__solver = AcadosOcpSolver(self.__ocp, json_file=json_file)
        self.__integrator = AcadosSimSolver(self.__ocp, json_file=json_file)
        #if self.__cythonsolver==True:
        if False:
            AcadosOcpSolver.generate(self.__ocp, json_file='acados_ocp.json')
            AcadosOcpSolver.build(self.__ocp.code_export_directory, with_cython=True)
            self.__solver = AcadosOcpSolver.create_cython_solver('acados_ocp.json')
    
    def controller(self,x_current,traj):

       
        x=x_current[0] 
        y=x_current[1]
        z=x_current[2]


        x_goal=traj[0]
        y_goal=traj[1]
        z_goal=traj[2]
        
        
        state = np.array([x,y,z])


        self.__set_controller_trajectory(x_goal,y_goal,z_goal)

        self.__solver.set(0, 'lbx', state)
        self.__solver.set(0, 'ubx', state)
        
        status = self.__solver.solve()
        if status != 0 :
            print('acados acados_ocp_solver returned status {}. Exiting.'.format(status))
            #raise Exception('acados acados_ocp_solver returned status {}. Exiting.'.format(status))
        
        if self.__print_status:
            self.__solver.print_statistics()
        self.solver_status=status

        u_list=[]
        x_list=[]

        for i in range(self.__N):
                
            u1=self.__solver.get(i, 'u')[0]
            u2=self.__solver.get(i, 'u')[1]
            u_list.append([u1,u2])
            u_list.append(self.__solver.get(i, 'u'))
            x_list.append(self.__solver.get(i,'x'))
 
            
        
        self.u_list=(u_list)
        self.x_list=(x_list)

        


        return self.u_list, self.x_list

    def __set_controller_trajectory(self,x_goal,y_goal,z_goal):
        Q=self.__Q
        for i in range(self.__N):
                self.__solver.cost_set(i, 'W', scipy.linalg.block_diag(Q, self.__R))
                self.__solver.set(i, 'yref', np.concatenate((np.array([x_goal,y_goal,z_goal]),
                                                             np.zeros((self.__nu)))))
        self.__solver.cost_set(self.__N, 'W', Q)
        self.__solver.set(self.__N, 'yref', np.array([x_goal,y_goal,z_goal]))     
        
    def __set_params(self,obs):

        params=0
            
        
                 
        return params
    
    def simulator(self,x,u):
        self.__integrator.set('x', x)
        self.__integrator.set('u', u)
        self.__integrator.solve()
        xcurrent=self.__integrator.get('x')
        return xcurrent.reshape(1,-1)
    