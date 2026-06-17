import numpy as np

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
    
link1_length = 2
link2_length = 1

# z_axis = np.array([0,0,1])
# x_axis = np.array([1,0,0])
link1 = Link(di=0, ai=link1_length, alphai=0, thetainit=0)
link2 = Link(di=0, ai=link2_length, alphai=0, thetainit=0)
print(link1.get_dh_matrix())
print(link2.get_dh_matrix())
print(link1.get_new_dh_matrix(np.pi/2))