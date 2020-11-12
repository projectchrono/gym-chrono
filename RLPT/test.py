import torch
import numpy as np
import Model

device   = torch.device("cpu") 
AC = Model.MultiSensorSimple([150,50], 4, 3)
image = np.ones([1,150,50,3])
sens2 = np.asarray([1,2,3,4])
data = (image, sens2)
state = [torch.FloatTensor(d).to(device) for d in data]
AC(state)
