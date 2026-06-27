import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from matplotlib.patches import Wedge, Circle

class Link:
#   def __init__(self, z_axis : np.array, x_axis : np.array, 
#                di : float, ai : float, alphai : float, thetainit : float):
    def __init__(self, di : float, ai : float,
                 alphai : float, thetainit : float):
        #self.z_axis = z_axis
        # self.x_axis= x_axis
        self.di = di
        self.ai = ai
        self.alphai = alphai
        self.thetai = thetainit
        
        self._di_matrix = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,self.di],[0,0,0,1]])
        self._thetai_matrix = np.array([[np.cos(self.thetai),-np.sin(self.thetai),0,0],
                                        [np.sin(self.thetai),np.cos(self.thetai),0,0],
                                        [0,0,1,0],[0,0,0,1]])
        self._ai_matrix = np.array([[1,0,0,self.ai],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        self._alphai_matrix = np.array([[1,0,0,0],
                                       [0,np.cos(self.alphai),-np.sin(self.alphai),0],
                                       [0,np.sin(self.alphai), np.cos(self.alphai),0],
                                       [0,0,0,1]])

        self.dh_matrix = (self._thetai_matrix @ self._di_matrix 
                        @ self._ai_matrix @ self._alphai_matrix)
        
    def get_dh_matrix(self):
        return self.dh_matrix
    def get_new_dh_matrix(self,new_thetai : float):
        self._new_thetai_matrix = np.array([[np.cos(new_thetai),-np.sin(new_thetai),0,0],
                                        [np.sin(new_thetai),np.cos(new_thetai),0,0],
                                        [0,0,1,0],[0,0,0,1]])
        self.new_dh_matrix = (self._new_thetai_matrix @ self._di_matrix 
                        @ self._ai_matrix @ self._alphai_matrix)
        return self.new_dh_matrix
    
class KinematicsChain:
    def __init__(self, link_list, rr=True):
        self.link_list = link_list
        self.l_no = len(link_list)
        self.origin=np.eye(4)
        self.rr = rr
    
    def get_final_forward_km_pos(self, input_list): 
        #get forwards kinematics position
        
        self.hom_matrix = self.origin
        for i in range(self.l_no):
            A_matrix = self.link_list[i].get_new_dh_matrix(input_list[i])
            self.hom_matrix = self.hom_matrix @ A_matrix
        return self.hom_matrix[0:3,3] 
    
    def get_all_forward_km_pos(self, input_list): 
        #get forwards kinematics position
        
        self.hom_matrix = self.origin
        self.pos_list = self.hom_matrix[0:3,3].reshape((1,3)) 
        for i in range(self.l_no):
            A_matrix = self.link_list[i].get_new_dh_matrix(input_list[i])
            self.hom_matrix = self.hom_matrix @ A_matrix
            self.pos_list = np.concatenate((self.pos_list, 
                                self.hom_matrix[0:3,3].reshape((1,3))), axis=0)           
        return self.pos_list
    
    
    def get_inverse_kinematics(self, target_pos):
        #get inverse kinematics position
        if(self.rr == False):
            return "can't do inverse kinematics for non rr robots as of now :("
        target_x = target_pos[0]
        target_y = target_pos[1]
        link1_length = self.link_list[0].ai
        link2_length = self.link_list[1].ai
        #calculate theta2
        cos_theta2 = (np.square(target_x) + np.square(target_y) - np.square(link1_length) 
                    - np.square(link2_length)) / (2 * link1_length * link2_length)
        if(cos_theta2 < -1 or cos_theta2 > 1):
            cos_theta2 = np.clip(cos_theta2, -1, 1)
        theta_2 = np.arccos(cos_theta2)
        #calculate theta1
        theta_1 = np.arctan2(target_y, target_x) - np.arctan2((link2_length * np.sin(theta_2)), 
                                                              (link1_length + link2_length * np.cos(theta_2)))
        if(-np.pi/2 <= theta_1 <= np.pi / 2 
           and -np.pi/2 <= theta_2 <= np.pi/2):
            return np.array([theta_1, theta_2])
        else: 
            "No solution found in bounds (-pi/2 to pi/2): {np.array2string(_solution, precision=3)}"
            _solution = np.array([theta_1, theta_2])
            return _solution
            

class ArmPlotter:
    def __init__(self, kinematics_chain): 
        'ASSUME ITS A RR ROBOT FOR NOW. 2DOF ONLY.'
        self.kinematics_chain = kinematics_chain 
    def plot_fm_update(self, val):
        input_list[0] = np.radians(self.theta1_slider.val)
        input_list[1] = np.radians(self.theta2_slider.val)
        all_pos = self.kinematics_chain.get_all_forward_km_pos(input_list)
        x_og = all_pos[:,0]
        y_og = all_pos[:,1]
        
        self.line1.set_xdata(x_og[0:2])
        self.line1.set_ydata(y_og[0:2])
        self.line2.set_xdata(x_og[1:])
        self.line2.set_ydata(y_og[1:])
        self.ee_dot.set_xdata(np.array([x_og[-1]]))
        self.ee_dot.set_ydata(np.array([y_og[-1]]))
        self.final_title = np.array2string(self.kinematics_chain.get_final_forward_km_pos(input_list), precision=3)
        self.title.set_text(
            f"Kinematics Simulation, end effector position: {self.final_title}")
        self.fig.canvas.draw_idle()

    
    def plot_fm(self, original_input_list=np.array([0,0])):
        link1_length = self.kinematics_chain.link_list[0].ai
        link2_length = self.kinematics_chain.link_list[1].ai
        all_pos = self.kinematics_chain.get_all_forward_km_pos(original_input_list)

        x_og = all_pos[:,0]
        y_og = all_pos[:,1]
        #z_og = all_pos[:,2]
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-link1_length-link2_length, link1_length+link2_length)
        self.ax.set_ylim(-link1_length-link2_length, link1_length+link2_length)
        self.ax.set_box_aspect(1)
        self.fig.subplots_adjust(bottom=0.25)

        self.line1, = self.ax.plot(x_og[0:2], y_og[0:2], 'o-', lw = 2, c='blue')
        self.line2, = self.ax.plot(x_og[1:], y_og[1:], 'o-', lw = 2, c='red')
        self.ee_dot, = self.ax.plot(x_og[-1], y_og[-1], 'o', c='green')
        self.title = self.ax.set_title('Kinematics Simulation, end effector position: ' 
                            + str(self.kinematics_chain.get_final_forward_km_pos(input_list)))
        
        self.axtheta1 = self.fig.add_axes((0.25,0.1,0.65,0.03))
        self.theta1_slider = Slider(
            ax=self.axtheta1,
            label="theta 1",
            valmin=-90,
            valmax=90,
            valinit=0
        )
        self.axtheta2 = self.fig.add_axes((0.25,0.05,0.65,0.03))
        self.theta2_slider = Slider(
            ax=self.axtheta2,
            label="theta 2",
            valmin=-90,
            valmax=90,
            valinit=0
        )
        self.theta1_slider.on_changed(self.plot_fm_update)
        self.theta2_slider.on_changed(self.plot_fm_update)
    
    def plot_im(self, original_input_list=np.array([0,0])):
        link1_length = self.kinematics_chain.link_list[0].ai
        link2_length = self.kinematics_chain.link_list[1].ai
        theta_inputs = self.kinematics_chain.get_inverse_kinematics(original_input_list)
        theta1 = theta_inputs[0]
        theta2 = theta_inputs[1]
        all_pos = self.kinematics_chain.get_all_forward_km_pos([theta1, theta2])

        x_og = all_pos[:,0]
        y_og = all_pos[:,1]
        #z_og = all_pos[:,2]
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-link1_length-link2_length, link1_length+link2_length)
        self.ax.set_ylim(-link1_length-link2_length, link1_length+link2_length)
        self.ax.set_box_aspect(1)
        self.fig.subplots_adjust(bottom=0.25)

        self.line1, = self.ax.plot(x_og[0:2], y_og[0:2], 'o-', lw = 2, c='blue')
        self.line2, = self.ax.plot(x_og[1:], y_og[1:], 'o-', lw = 2, c='red')
        self.ee_dot = Circle((x_og[-1], y_og[-1]), 0.1, color='green', alpha=0.5)
        self.title = self.ax.set_title('Kinematics Simulation, EE position: ' 
                            + str(np.array2string(np.array(original_input_list), precision=2))
                            + '\n theta1: '+f'{np.degrees(theta1):.2f}' +
                            ', theta2: ' + f'{np.degrees(theta2):.2f}')
        
        self.dragging = False
        self.point = self.ax.add_patch(self.ee_dot)
        self.invkm_connect()

        if(theta1 <0):
            self._angle_annotation_t1 = Wedge((x_og[0], y_og[0]), 0.5, theta1=np.degrees(theta1), theta2=0, color='blue', alpha=0.3)
        else:
            self._angle_annotation_t1 = Wedge((x_og[0], y_og[0]), 0.5, theta1=0, theta2=np.degrees(theta1), color='blue', alpha=0.3)
        if(theta2 <theta1):
            self._angle_annotation_t2 = Wedge((x_og[1], y_og[1]), 0.5, theta1=np.degrees(theta1+theta2), theta2=np.degrees(theta1), color='red', alpha=0.3)
        else:
            self._angle_annotation_t2 = Wedge((x_og[1], y_og[1]), 0.5, theta1=np.degrees(theta1), theta2=np.degrees(theta1+theta2), color='red', alpha=0.3)
        
        self.angle_annotation_t1 = self.ax.add_patch(self._angle_annotation_t1)
        self.angle_annotation_t2 = self.ax.add_patch(self._angle_annotation_t2)

    def invkm_near_point(self, x1, y1, x2, y2, threshold=0.1):
        return np.sqrt((x1 - x2)**2 + (y1 - y2)**2) <= threshold
    
    def invkm_clamp_to_bounds(self, x, y):
        thetaoutputs = self.kinematics_chain.get_inverse_kinematics(np.array([x,y]))
        theta1 = thetaoutputs[0]
        theta2 = thetaoutputs[1]
        theta1=np.clip(theta1, -np.pi/2, np.pi/2)
        theta2=np.clip(theta2, -np.pi/2, np.pi/2)
        return self.kinematics_chain.get_final_forward_km_pos([theta1, theta2])[0:2]
    
    def invkm_update(self,x,y):
        # update leg segments based on inverse kinematics
        # self.point.center = (event.xdata, event.ydata)
        # change all angles based on inv km
        thetaoutputs = self.kinematics_chain.get_inverse_kinematics(np.array([x, y]))
        all_pos = self.kinematics_chain.get_all_forward_km_pos(thetaoutputs)
        x_og = all_pos[:,0]
        y_og = all_pos[:,1]
        
        self.line1.set_xdata(x_og[0:2])
        self.line1.set_ydata(y_og[0:2])
        self.line2.set_xdata(x_og[1:])
        self.line2.set_ydata(y_og[1:])
        self.point.set_center((x_og[-1], y_og[-1]))
        self.final_title = np.array2string(thetaoutputs, precision=3)
        self.title.set_text(
            f"Kinematics Simulation, theta values: {self.final_title}")
        
        theta1 = thetaoutputs[0]
        theta2 = thetaoutputs[1]
        self.angle_annotation_t1.set_center((x_og[0], y_og[0]))
        self.angle_annotation_t2.set_center((x_og[1], y_og[1]))
        if(theta1 <0):
            self.angle_annotation_t1.set_theta1 = np.degrees(theta1)
            self.angle_annotation_t1.set_theta2 = 0
        else:
            self.angle_annotation_t1.set_theta1 = 0
            self.angle_annotation_t1.set_theta2 = np.degrees(theta1)
            
        if(theta2 <theta1):
            self.angle_annotation_t2.set_theta1 = np.degrees(theta1+theta2)
            self.angle_annotation_t2.set_theta2 = np.degrees(theta1)
        else:
            self.angle_annotation_t2.set_theta1 = np.degrees(theta1)
            self.angle_annotation_t2.set_theta2 = np.degrees(theta1+theta2)
        self.fig.canvas.draw_idle()

    def invkm_connect(self):
        'connect to all the events we need'
        self.cidpress = self.point.figure.canvas.mpl_connect(
            'button_press_event', self.invkm_on_press)
        self.cidrelease = self.point.figure.canvas.mpl_connect(
            'button_release_event', self.invkm_on_release)
        self.cidmotion = self.point.figure.canvas.mpl_connect(
            'motion_notify_event', self.invkm_on_motion)
    
    def invkm_disconnect(self):
        'disconnect all the stored connection ids'
        self.point.figure.canvas.mpl_disconnect(self.cidpress)
        self.point.figure.canvas.mpl_disconnect(self.cidrelease)
        self.point.figure.canvas.mpl_disconnect(self.cidmotion)
    
    def invkm_on_press(self, event):
        if event.inaxes != self.point.axes: return
        if self.invkm_near_point(event.xdata, event.ydata, self.point.center[0], self.point.center[1]):
            self.dragging = True
        else:
            return
    def invkm_on_motion(self, event):
        if not self.dragging: return
        if event.inaxes != self.point.axes: return
        # update arm with fwd km on clamped x,y
        x, y = event.xdata, event.ydata
        x, y = self.invkm_clamp_to_bounds(x,y)
        self.invkm_update(x, y)

    def invkm_on_release(self, event):
        self.dragging = False
link1_length = 2
link2_length = 1
# z_axis = np.array([0,0,1])
# x_axis = np.array([1,0,0])
link1 = Link(di=0, ai=link1_length, alphai=0, thetainit=0)
link2 = Link(di=0, ai=link2_length, alphai=0, thetainit=0)
list_of_links = [link1, link2]
kinematics_chain = KinematicsChain(list_of_links)
input_list = [0, 0]
#print(kinematics_chain.get_final_forward_km_pos(input_list))
arm_plotter = ArmPlotter(kinematics_chain)
arm_plotter.plot_im(original_input_list=[2.25,1.25])
plt.show()