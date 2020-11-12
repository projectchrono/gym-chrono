import torch
import torch.nn as nn
from torch.distributions import Normal
import copy

def init_weights(m):
    if isinstance(m, nn.Linear):
        nn.init.normal_(m.weight, mean=0., std=0.1)
        nn.init.constant_(m.bias, 0.1)
    if isinstance(m, nn.Conv2d):
        nn.init.xavier_normal_(m.weight)
        nn.init.constant_(m.bias, 0.0)

class ActorCritic(nn.Module):
    def __init__(self, num_inputs, num_outputs, hidden_size, std=0.0):
        super(ActorCritic, self).__init__()

        self.critic = nn.Sequential(
            nn.Linear(num_inputs, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, 1)
        )

        self.actor = nn.Sequential(
            nn.Linear(num_inputs, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, num_outputs),
            nn.Tanh()
        )
        self.log_std = nn.Parameter(torch.ones(1, num_outputs) * std)
        #self.log_std = self.log_std.clamp(-20, -1)

        self.apply(init_weights)

    def forward(self, x):
        value = self.critic(x)
        mu = self.actor(x)
        std = self.log_std.exp().expand_as(mu)
        dist = Normal(mu, std)
        return dist, value


class ActorCriticPCoady(ActorCritic):
    def __init__(self, num_inputs, num_outputs, std=-.5):
        nn.Module.__init__(self)
        h1 = num_inputs * 10
        h3 = num_outputs * 10
        h2 = int(pow(h1*h3,0.5))
        h2_cr = int(pow(h1*10,0.5))
        self.critic = nn.Sequential(
            nn.Linear(num_inputs, h1),
            nn.Tanh(),
            nn.Linear(h1, h2_cr),
            nn.Tanh(),
            nn.Linear(h2_cr, 10),
            nn.Tanh(),
            nn.Linear(10, 1)
        )

        self.actor = nn.Sequential(
            nn.Linear(num_inputs, h1),
            nn.Tanh(),
            nn.Linear(h1, h2),
            nn.Tanh(),
            nn.Linear(h2, h3),
            nn.Tanh(),
            nn.Linear(h3, num_outputs),
            nn.Tanh()
        )
        self.log_std = nn.Parameter(torch.ones(1, num_outputs) * std)

        self.apply(init_weights)

# From baselines Policies.py , line 15

def outputSize(in_size, kernel_size, stride, padding):
    conv_size = copy.deepcopy(in_size)
    for i in range(len(kernel_size)):
        conv_size[0] = int((conv_size[0] - kernel_size[i] + 2*(padding[i])) / stride[i]) + 1
        conv_size[1] = int((conv_size[1] - kernel_size[i] + 2*(padding[i])) / stride[i]) + 1

    return(conv_size)

class Flatten(torch.nn.Module):
    def forward(self, x):
        batch_size = x.shape[0]
        return x.view(batch_size, -1)

class ActorCritic_nature_cnn(ActorCritic):
    # CNN from Nature paper.
    def __init__(self, image_shape, num_outputs, std=-.5):
        super(ActorCritic, self).__init__()
        self.input_shape = image_shape
        fc_size = outputSize(image_shape, [8,4,3], [4,2,1], [0,0,0])
        self.actor = nn.Sequential(
            nn.Conv2d(3, 32, kernel_size=8, stride=4),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=4, stride=2),
            nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=1),
            nn.ReLU(),
            Flatten(),
            nn.Linear(fc_size[0] * fc_size[1] * 64, num_outputs),
            nn.Tanh()
        )

        self.critic = nn.Sequential(
            nn.Conv2d(3, 32, kernel_size=8, stride=4),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=4, stride=2),
            nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=1),
            nn.ReLU(),
            Flatten(),
            nn.Linear(fc_size[0] * fc_size[1] * 64, 1)
        )

        self.log_std = nn.Parameter(torch.ones(1, num_outputs) * std)

        self.apply(init_weights)


    def forward(self, x):
        # Input shape is [batch, Height, Width, RGB] Torch wants [batch, RGB, Height, Width]. Must Permute
        x = ((x-127)/255).permute(0,3,1,2)
        value = self.critic(x)
        mu = self.actor(x)#.mul_(2)).add_(-1)
        std = self.log_std.exp().expand_as(mu)
        dist = Normal(mu, std)
        return dist, value


class MultiSensorSimple(nn.Module):
    def __init__(self, image_shape, sens2_shape, num_outputs, std=-0.5):
        super(MultiSensorSimple, self).__init__()
        self.input_shape = image_shape
        fc_size = outputSize(image_shape, [8, 4, 3], [4, 2, 1], [0, 0, 0])
        self.actor_cnn = nn.Sequential(
            nn.Conv2d(3, 32, kernel_size=8, stride=4),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=4, stride=2),
            nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=1),
            Flatten(),
            nn.ReLU(),
            nn.Linear(fc_size[0] * fc_size[1] * 64, num_outputs * 5),

        )
        self.actor_fc0 = nn.Linear(sens2_shape, num_outputs * 5)
        self.actor_fc1 = nn.Linear(num_outputs * 10, num_outputs * 5)
        self.actor_fc2 = nn.Linear(num_outputs * 5, num_outputs)

        self.critic_cnn = nn.Sequential(
            nn.Conv2d(3, 32, kernel_size=8, stride=4),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=4, stride=2),
            nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=1),
            Flatten(),
            nn.ReLU(),
            nn.Linear(fc_size[0] * fc_size[1] * 64, num_outputs * 5),

        )
        self.critic_fc0 = nn.Linear(sens2_shape, num_outputs * 5)
        self.critic_fc1 = nn.Linear(num_outputs * 10, num_outputs * 5)
        self.critic_fc2 = nn.Linear(num_outputs * 5, 1)


        self.log_std = nn.Parameter(torch.ones(1, num_outputs) * std)
        self.apply(init_weights)


    def forward(self, data):
        x0 = ((data[0]-127)/255.).permute(0, 3, 1, 2)
        x1 = self.actor_cnn(x0)#.view(-1)
        x2 = nn.functional.relu(self.actor_fc0(data[1]))
        x = torch.cat((x1, x2), dim=1)
        x = nn.functional.relu(self.actor_fc1(x))
        x = self.actor_fc2(x)
        mu = torch.tanh(x)
        std = self.log_std.exp().expand_as(mu)
        dist = Normal(mu, std)

        y1 = self.critic_cnn(x0)#.view(-1)
        y2 = nn.functional.relu(self.critic_fc0(data[1]))
        y = torch.cat((y1, y2), dim=1)
        y = nn.functional.relu(self.critic_fc1(y))
        value = self.critic_fc2(y)

        return dist, value

    def export(self,filename):
        inference_model = MultiSensorSimpleInference(self.input_shape,self.sens2_shape,self.num_outputs).cuda()
        # inference_model.parameters() = self.actor_cnn.parameters()

        inference_model.load_state_dict(self.state_dict())

        tmp_input = [torch.ones(1, self.input_shape[0], self.input_shape[1], 3).cuda(), torch.ones(1, self.sens2_shape).cuda()]
        print("EXPORTING ONNX MODEL...")
        torch.onnx.export(inference_model,               # model being run
                          tmp_input,
                          filename,                  # where to save the model (can be a file or file-like object)
                          export_params=True,        # store the trained parameter weights inside the model file
                          opset_version=10,          # the ONNX version to export the model to
                          do_constant_folding=True,  # whether to execute constant folding for optimization
                          input_names = ['input1',"input2"],   # input layer names
                          output_names = ["output"]) # output layer names
        print("MODEL EXPORTED!")
        pass

class MultiSensorSimpleInference(nn.Module):
    def __init__(self, image_shape, sens2_shape, num_outputs, std=-0.5):
        super(MultiSensorSimpleInference, self).__init__()
        self.input_shape = image_shape
        fc_size = outputSize(image_shape, [8, 4, 3], [4, 2, 1], [0, 0, 0])
        self.actor_cnn = nn.Sequential(
            nn.Conv2d(3, 32, kernel_size=8, stride=4),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=4, stride=2),
            nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=1),
            Flatten(),
            nn.ReLU(),
            nn.Linear(fc_size[0] * fc_size[1] * 64, num_outputs * 5),

        )
        self.actor_fc0 = nn.Linear(sens2_shape, num_outputs * 5)
        self.actor_fc1 = nn.Linear(num_outputs * 10, num_outputs * 5)
        self.actor_fc2 = nn.Linear(num_outputs * 5, num_outputs)

        self.critic_cnn = nn.Sequential(
            nn.Conv2d(3, 32, kernel_size=8, stride=4),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=4, stride=2),
            nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=1),
            Flatten(),
            nn.ReLU(),
            nn.Linear(fc_size[0] * fc_size[1] * 64, num_outputs * 5),

        )
        self.critic_fc0 = nn.Linear(sens2_shape, num_outputs * 5)
        self.critic_fc1 = nn.Linear(num_outputs * 10, num_outputs * 5)
        self.critic_fc2 = nn.Linear(num_outputs * 5, 1)


        self.log_std = nn.Parameter(torch.ones(1, num_outputs) * std)

    def forward(self, data):
        x0 = ((data[0]-127)/255.).permute(0, 3, 1, 2)
        x1 = self.actor_cnn(x0)#.view(-1)
        x2 = nn.functional.relu(self.actor_fc0(data[1]))
        x = torch.cat((x1, x2), dim=1)
        x = nn.functional.relu(self.actor_fc1(x))
        x = self.actor_fc2(x)
        mu = torch.tanh(x)

        return mu

class MultiSensorLateFusion(nn.Module):
    def __init__(self, image_shape, sens2_shape, num_outputs, std=-0.5):
        super(MultiSensorLateFusion, self).__init__()
        self.input_shape = image_shape
        fc_size = outputSize(image_shape, [8, 4, 3], [4, 2, 1], [0, 0, 0])

        self.actor_cnn = nn.Sequential(
            nn.Conv2d(3, 32, kernel_size=8, stride=4),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=4, stride=2),
            nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=1),
            Flatten(),
            nn.ReLU(),
            nn.Linear(fc_size[0] * fc_size[1] * 64, num_outputs),
            nn.ReLU()
        )
        self.actorMLP = nn.Sequential(
            nn.Linear(sens2_shape, sens2_shape*5),
            nn.ReLU(),
            nn.Linear(sens2_shape*5, (num_outputs+sens2_shape)*5),
            nn.ReLU(),
            nn.Linear((num_outputs+sens2_shape)*5, num_outputs*5),
            nn.ReLU(),
            nn.Linear(num_outputs*5, num_outputs),
            nn.ReLU()
        )
        self.actorOut = nn.Sequential(
            nn.Linear(num_outputs * 2, num_outputs),
            nn.Tanh()
        )

        self.critic_cnn = nn.Sequential(
            nn.Conv2d(3, 32, kernel_size=8, stride=4),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=4, stride=2),
            nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=1),
            Flatten(),
            nn.ReLU(),
            nn.Linear(fc_size[0] * fc_size[1] * 64, 1),
            nn.ReLU()
        )
        self.criticMLP = nn.Sequential(
            nn.Linear(sens2_shape, sens2_shape*5),
            nn.ReLU(),
            nn.Linear(sens2_shape*5, sens2_shape*5),
            nn.ReLU(),
            nn.Linear(sens2_shape*5, 5),
            nn.ReLU(),
            nn.Linear(5, 1),
            nn.ReLU()
        )
        self.criticOut = nn.Sequential(
            nn.Linear(2, num_outputs),
        )

        self.log_std = nn.Parameter(torch.ones(1, num_outputs) * std)
        self.apply(init_weights)


    def forward(self, data):
        x0 = ((data[0]-127)/255.).permute(0, 3, 1, 2)
        x1 = self.actor_cnn(x0)
        x2 = self.actorMLP(data[1])
        x = torch.cat((x1, x2), dim=1)
        mu = self.actorOut(x)
        std = self.log_std.exp().expand_as(mu)
        dist = Normal(mu, std)

        y1 = self.critic_cnn(x0)
        y2 = self.criticMLP(data[1])
        y = torch.cat((y1, y2), dim=1)
        value = self.criticOut(y)

        return dist, value

    def export(self,filename):
        inference_model = MultiSensorLateFusionInference(self.input_shape,self.sens2_shape,self.num_outputs).cuda()
        # inference_model.parameters() = self.actor_cnn.parameters()

        inference_model.load_state_dict(self.state_dict())

        tmp_input = [torch.ones(1, self.input_shape[0], self.input_shape[1], 3).cuda(), torch.ones(1, self.sens2_shape).cuda()]
        print("EXPORTING ONNX MODEL...")
        torch.onnx.export(inference_model,               # model being run
                          tmp_input,
                          filename,                  # where to save the model (can be a file or file-like object)
                          export_params=True,        # store the trained parameter weights inside the model file
                          opset_version=10,          # the ONNX version to export the model to
                          do_constant_folding=True,  # whether to execute constant folding for optimization
                          input_names = ['input1',"input2"],   # input layer names
                          output_names = ["output"]) # output layer names
        print("MODEL EXPORTED!")
        pass

class MultiSensorLateFusionInference(nn.Module):
    def __init__(self, image_shape, sens2_shape, num_outputs, std=-0.5):
        super(MultiSensorLateFusionInference, self).__init__()
        self.input_shape = image_shape
        fc_size = outputSize(image_shape, [8, 4, 3], [4, 2, 1], [0, 0, 0])

        self.actor_cnn = nn.Sequential(
            nn.Conv2d(3, 32, kernel_size=8, stride=4),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=4, stride=2),
            nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=1),
            Flatten(),
            nn.ReLU(),
            nn.Linear(fc_size[0] * fc_size[1] * 64, num_outputs),
            nn.ReLU()
        )
        self.actorMLP = nn.Sequential(
            nn.Linear(sens2_shape, sens2_shape*5),
            nn.ReLU(),
            nn.Linear(sens2_shape*5, (num_outputs+sens2_shape)*5),
            nn.ReLU(),
            nn.Linear((num_outputs+sens2_shape)*5, num_outputs*5),
            nn.ReLU(),
            nn.Linear(num_outputs*5, num_outputs),
            nn.ReLU()
        )
        self.actorOut = nn.Sequential(
            nn.Linear(num_outputs * 2, num_outputs),
            nn.Tanh()
        )

        self.critic_cnn = nn.Sequential(
            nn.Conv2d(3, 32, kernel_size=8, stride=4),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=4, stride=2),
            nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=1),
            Flatten(),
            nn.ReLU(),
            nn.Linear(fc_size[0] * fc_size[1] * 64, 1),
            nn.ReLU()
        )
        self.criticMLP = nn.Sequential(
            nn.Linear(sens2_shape, sens2_shape*5),
            nn.ReLU(),
            nn.Linear(sens2_shape*5, sens2_shape*5),
            nn.ReLU(),
            nn.Linear(sens2_shape*5, 5),
            nn.ReLU(),
            nn.Linear(5, 1),
            nn.ReLU()
        )
        self.criticOut = nn.Sequential(
            nn.Linear(2, num_outputs),
        )

        self.log_std = nn.Parameter(torch.ones(1, num_outputs) * std)
        self.apply(init_weights)


    def forward(self, data):
        x0 = ((data[0]-127)/255.).permute(0, 3, 1, 2)
        x1 = self.actor_cnn(x0)
        x2 = self.actorMLP(data[1])
        x = torch.cat((x1, x2), dim=1)
        mu = self.actorOut(x)

        return mu

class MultiSensorLSTM(nn.Module):
    def __init__(self, image_shape, sens2_shape, num_outputs, std=-0.5):
        super(MultiSensorLSTM, self).__init__()
        self.input_shape = image_shape
        fc_size = outputSize(image_shape, [8, 4, 3], [4, 2, 1], [0, 0, 0])
        self.actor_cnn = nn.Sequential(
            nn.Conv2d(3, 32, kernel_size=8, stride=4),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=4, stride=2),
            nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=1),
            Flatten(),
            nn.ReLU(),
            nn.Linear(fc_size[0] * fc_size[1] * 64, num_outputs * 5),

        )
        self.actor_LSTM = nn.LSTM(sens2_shape, num_outputs * 5, num_layers=1)
        self.actor_fc1 = nn.Linear(num_outputs * 10, num_outputs * 5)
        self.actor_fc2 = nn.Linear(num_outputs * 5, num_outputs)

        self.critic_cnn = nn.Sequential(
            nn.Conv2d(3, 32, kernel_size=8, stride=4),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=4, stride=2),
            nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=1),
            Flatten(),
            nn.ReLU(),
            nn.Linear(fc_size[0] * fc_size[1] * 64, num_outputs * 5),

        )
        self.critic_LSTM = nn.LSTM(sens2_shape, num_outputs * 5, num_layers=1)
        self.critic_fc1 = nn.Linear(num_outputs * 10, num_outputs * 5)
        self.critic_fc2 = nn.Linear(num_outputs * 5, 1)

        self.log_std = nn.Parameter(torch.ones(1, num_outputs) * std)
        self.apply(init_weights)

    def forward(self, data):
        x0 = ((data[0]-127)/255.).permute(0, 3, 1, 2)
        x1 = self.actor_cnn(x0)#.view(-1)
        x2 = self.actor_LSTM(data[1].view(1, data[1].shape[0], data[1].shape[1]))
        x = torch.cat((x1, x2[0].view(data[1].shape[0],-1)), dim=1)
        x = nn.functional.relu(self.actor_fc1(x))
        x = self.actor_fc2(x)
        mu = torch.tanh(x)
        std = self.log_std.exp().expand_as(mu)
        dist = Normal(mu, std)

        y1 = self.critic_cnn(x0)#.view(-1)
        y2 = self.critic_LSTM(data[1].view(1, data[1].shape[0], data[1].shape[1]))
        y = torch.cat((y1, y2[0].view(data[1].shape[0],-1)), dim=1)
        y = nn.functional.relu(self.critic_fc1(y))
        value = self.critic_fc2(y)

        return dist, value

class MultiSensorEarlyFusion(nn.Module):
    def __init__(self, image_shape, sens2_shape, num_outputs, std=-0.5):
        super(MultiSensorEarlyFusion, self).__init__()
        self.input_shape = image_shape
        self.sens2_shape = sens2_shape
        self.num_outputs = num_outputs
        fc_size = outputSize(image_shape, [8, 4, 3], [4, 2, 1], [0, 0, 0])
        self.actor_cnn = nn.Sequential(
            nn.Conv2d(3, 32, kernel_size=8, stride=4),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=4, stride=2),
            nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=1),
            Flatten(),
            nn.ReLU(),
            nn.Linear(fc_size[0] * fc_size[1] * 64, num_outputs * 5),

        )
        self.actor_fc0 = nn.Linear(sens2_shape, num_outputs * 5)
        self.actor_fc1 = nn.Linear(num_outputs * 10, num_outputs * 20)
        self.actor_fc2 = nn.Linear(num_outputs * 20, num_outputs * 10)
        self.actor_fc3 = nn.Linear(num_outputs * 10, num_outputs * 5)
        self.actor_fc4 = nn.Linear(num_outputs * 5, num_outputs)

        self.critic_cnn = nn.Sequential(
            nn.Conv2d(3, 32, kernel_size=8, stride=4),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=4, stride=2),
            nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=1),
            Flatten(),
            nn.ReLU(),
            nn.Linear(fc_size[0] * fc_size[1] * 64, num_outputs * 5),

        )
        self.critic_fc0 = nn.Linear(sens2_shape, num_outputs * 5)
        self.critic_fc1 = nn.Linear(num_outputs * 10, num_outputs * 20)
        self.critic_fc2 = nn.Linear(num_outputs * 20, num_outputs * 10)
        self.critic_fc3 = nn.Linear(num_outputs * 10, num_outputs * 5)
        self.critic_fc4 = nn.Linear(num_outputs * 5, 1)


        self.log_std = nn.Parameter(torch.ones(1, num_outputs) * std)
        self.apply(init_weights)


    def forward(self, data):
        x0 = ((data[0]-127)/255.).permute(0, 3, 1, 2)
        x1 = self.actor_cnn(x0)#.view(-1)
        x2 = nn.functional.relu(self.actor_fc0(data[1]))
        x = torch.cat((x1, x2), dim=1)
        x = nn.functional.relu(self.actor_fc1(x))
        x = nn.functional.relu(self.actor_fc2(x))
        x = nn.functional.relu(self.actor_fc3(x))
        mu = torch.tanh(self.actor_fc4(x))
        std = self.log_std.exp().expand_as(mu)
        dist = Normal(mu, std)

        y1 = self.critic_cnn(x0)#.view(-1)
        y2 = nn.functional.relu(self.critic_fc0(data[1]))
        y = torch.cat((y1, y2), dim=1)
        y = nn.functional.relu(self.critic_fc1(y))
        y = nn.functional.relu(self.critic_fc2(y))
        y = nn.functional.relu(self.critic_fc3(y))
        value = self.critic_fc4(y)

        return dist, value

    def export(self,filename):
        inference_model = MultiSensorEarlyFusionInference(self.input_shape,self.sens2_shape,self.num_outputs).cuda()
        # inference_model.parameters() = self.actor_cnn.parameters()

        inference_model.load_state_dict(self.state_dict())

        tmp_input = [torch.ones(1, self.input_shape[1], self.input_shape[0], 3).cuda(), torch.ones(1, self.sens2_shape).cuda()]
        print("EXPORTING ONNX MODEL...")
        torch.onnx.export(inference_model,               # model being run
                          tmp_input,
                          filename,                  # where to save the model (can be a file or file-like object)
                          export_params=True,        # store the trained parameter weights inside the model file
                          opset_version=10,          # the ONNX version to export the model to
                          do_constant_folding=True,  # whether to execute constant folding for optimization
                          input_names = ['input1',"input2"],   # input layer names
                          output_names = ["output"]) # output layer names
        print("MODEL EXPORTED!")
        pass

class MultiSensorEarlyFusionInference(nn.Module):
    def __init__(self, image_shape, sens2_shape, num_outputs, std=-0.5):
        super(MultiSensorEarlyFusionInference, self).__init__()
        self.input_shape = image_shape
        self.sens2_shape = sens2_shape
        self.num_outputs = num_outputs
        fc_size = outputSize(image_shape, [8, 4, 3], [4, 2, 1], [0, 0, 0])
        self.actor_cnn = nn.Sequential(
            nn.Conv2d(3, 32, kernel_size=8, stride=4),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=4, stride=2),
            nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=1),
            Flatten(),
            nn.ReLU(),
            nn.Linear(fc_size[0] * fc_size[1] * 64, num_outputs * 5),

        )
        self.actor_fc0 = nn.Linear(sens2_shape, num_outputs * 5)
        self.actor_fc1 = nn.Linear(num_outputs * 10, num_outputs * 20)
        self.actor_fc2 = nn.Linear(num_outputs * 20, num_outputs * 10)
        self.actor_fc3 = nn.Linear(num_outputs * 10, num_outputs * 5)
        self.actor_fc4 = nn.Linear(num_outputs * 5, num_outputs)

        self.critic_cnn = nn.Sequential(
            nn.Conv2d(3, 32, kernel_size=8, stride=4),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=4, stride=2),
            nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=1),
            Flatten(),
            nn.ReLU(),
            nn.Linear(fc_size[0] * fc_size[1] * 64, num_outputs * 5),

        )
        self.critic_fc0 = nn.Linear(sens2_shape, num_outputs * 5)
        self.critic_fc1 = nn.Linear(num_outputs * 10, num_outputs * 20)
        self.critic_fc2 = nn.Linear(num_outputs * 20, num_outputs * 10)
        self.critic_fc3 = nn.Linear(num_outputs * 10, num_outputs * 5)
        self.critic_fc4 = nn.Linear(num_outputs * 5, 1)


        self.log_std = nn.Parameter(torch.ones(1, num_outputs) * std)
        self.apply(init_weights)

    def forward(self, data):
        # data[0] = torch.reshape(data[0], (1, self.input_shape[1], self.input_shape[0], 3))
        x0 = ((data[0]-127)/255.).permute(0, 3, 1, 2)
        x1 = self.actor_cnn(x0)#.view(-1)
        x2 = nn.functional.relu(self.actor_fc0(data[1]))
        x = torch.cat((x1, x2), dim=1)
        x = nn.functional.relu(self.actor_fc1(x))
        x = nn.functional.relu(self.actor_fc2(x))
        x = nn.functional.relu(self.actor_fc3(x))
        mu = torch.tanh(self.actor_fc4(x))

        return mu
