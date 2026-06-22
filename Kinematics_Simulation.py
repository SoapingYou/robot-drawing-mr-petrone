import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

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
    def __init__(self, link_list):
        self.link_list = link_list
        self.l_no = len(link_list)
        self.origin=np.eye(4)
    
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

# now now im going to do most of the coding
# do you hear me mr ai autocomplete my code bro

all_pos = kinematics_chain.get_all_forward_km_pos(input_list)

x_og = all_pos[:,0]
y_og = all_pos[:,1]
#z_og = all_pos[:,2]
fig, ax = plt.subplots()
ax.set_xlim(-link1_length-link2_length, link1_length+link2_length)
ax.set_ylim(-link1_length-link2_length, link1_length+link2_length)
ax.set_box_aspect(1)
fig.subplots_adjust(bottom=0.25)
axtheta1 = fig.add_axes((0.25,0.1,0.65,0.03))
theta1_slider = Slider(
    ax=axtheta1,
    label="theta 1",
    valmin=-90,
    valmax=90,
    valinit=0
)
axtheta2 = fig.add_axes((0.25,0.05,0.65,0.03))
theta2_slider = Slider(
    ax=axtheta2,
    label="theta 2",
    valmin=-90,
    valmax=90,
    valinit=0
)
#for the bloody sake of time, im going to just assume
#2 links for now
line1, = ax.plot(x_og[0:2], y_og[0:2], 'o-', lw = 2, c='blue')
line2, = ax.plot(x_og[1:], y_og[1:], 'o-', lw = 2, c='red')
ee_dot, = ax.plot(x_og[-1], y_og[-1], 'o', c='green')
title = ax.set_title('Kinematics Simulation, end effector position: ' 
                     + str(kinematics_chain.get_final_forward_km_pos(input_list)))

def update(val):
    input_list[0] = np.radians(theta1_slider.val)
    input_list[1] = np.radians(theta2_slider.val)
    all_pos = kinematics_chain.get_all_forward_km_pos(input_list)
    x_og = all_pos[:,0]
    y_og = all_pos[:,1]
    
    line1.set_xdata(x_og[0:2])
    line1.set_ydata(y_og[0:2])
    line2.set_xdata(x_og[1:])
    line2.set_ydata(y_og[1:])
    ee_dot.set_xdata(x_og[-1])
    ee_dot.set_ydata(y_og[-1])
    title.set_text('Kinematics Simulation, end effector position: ' 
                     + str(kinematics_chain.get_final_forward_km_pos(input_list)))
    fig.canvas.draw_idle()


theta1_slider.on_changed(update)
theta2_slider.on_changed(update)

plt.show()