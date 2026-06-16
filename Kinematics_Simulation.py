import numpy as np

class RotationJoint:
    def __init__(self, z_axis : np.array, x_axis : np.array):
        self.z_axis = z_axis
        self.x_axis= x_axis
        

class BaseJoint(RotationJoint):
    def __init__(self, z_axis : np.array, x_axis : np.array):
        self.z_axis = z_axis
        self.x_axis= x_axis


class OtherJoint(RotationJoint):
    def __init__(self, z_axis : np.array, x_axis : np.array, 
                 di : float, ai : float, alphai : float, thetainit : float):
        #self.z_axis = z_axis
        # self.x_axis= x_axis
        self.di = di
        self.ai = ai
        self.alphai = alphai
        self.thetai = thetainit
        
        self._di_matrix = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,self.di],[0,0,0,1]])
        self._thetai_matrix = np.array([[np.cos(self.thetai),-np.sin(self.thetai),0,0],
                                        [np.sin(self.thetai),np.cos(self.tehtai),0,0],
                                        [0,0,1,0],[0,0,0,1]])
        self._ai_matrix = np.array([[1,0,0,self.ai],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        self.alphai_matrix = np.array([[1,0,0,0],
                                       [0,np.cos(self.alphai),-np.sin(self.alphai),0],
                                       [0,np.sin(self.alphai), np.cos(self.alphai),0],
                                       [0,0,0,1]])

        self.dh_matrix = (self._thetai_matrix @ self._di_matrix 
                        @ self._ai_matrix @ self._alphai_matrix)
        
        def get_dh_matrix(self):
            return self.dh_matrix
    

z_axis = np.array([0,0,1])
x_axis = np.array([1,0,0])