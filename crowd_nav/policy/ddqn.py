import torch
import torch.nn
import torch.optim as optim
import torch.nn.functional as F
import numpy as np
import itertools
import logging
from crowd_sim.envs.policy.policy import Policy
from crowd_sim.envs.utils.action import ActionRot, ActionXY
from crowd_sim.envs.utils.state import ObservableState, FullState


class Network(nn.Module):
    def __init__(self, input_channels, outputs, batch_size, device):
        super(Network, self).__init__()
        self.conv1 = nn.Conv2d(input_channels, 32, kernel_size=32)
        self.conv2 = nn.Conv2d(32, 64, kernel_size=32)
        self.conv3 = nn.Conv2d(64, 64, kernel_size=8)
        self.conv4 = nn.Conv2d(64, 64, kernel_size=8)
        self.fc1 = nn.Linear(4, 67)
        self.fc2 = nn.Linear(6400, 512)
        self.fc3 = nn.Linear(512, 512)
        self.fc4_ea = nn.Linear(512, outputs) # Elements A(s,a) that depend on your actions.
        self.fc4_ev = nn.Linear(512, 1) # Elements V(s) that are determined only by the state.
        self.batch_size = batch_size
        self.device = device

    def forward(self, state_m, state_g, state_v):
        # print("state_m.size() = ", state_m.size())
        # print("state_g.size() = ", state_g.size())
        # print("state_v.size() = ", state_v.size())
        x_m = F.relu(self.conv1(state_m))
        # print("conv1 -> x_m.size() = ", x_m.size())
        x_m = F.relu(self.conv2(x_m))
        # print("conv2 -> x_m.size() = ", x_m.size())
        x_m = F.relu(self.conv3(x_m))
        # print("conv3 -> x_m.size() = ", x_m.size())

        x_gv_ = torch.cat((state_g, state_v), 1)
        x_gv_ = F.relu(self.fc1(x_gv_))
        # print("x_gv_.size() = ", x_gv_.size())
        convw = x_m.shape[2]
        convh = x_m.shape[3]
        # print("convw =", convw, ", convh =", convh)
        x_gv = torch.empty(self.batch_size, convw, convh)
        
        grid_num = convw
        for i in range(grid_num):
            value = x_gv_[0][i].item()
            tile_gv = torch.full((convw, convh), value)
            x_gv = torch.stack(tuple(tile_gv), 0)
        
        # print("x_gv.size() = ", x_gv.size())

        x_m = x_m.to('cpu')
        x_gv = x_gv.to('cpu')
        x_pls = x_m + x_gv
        # x_pls = x_pls.to(device)
        x_pls = x_pls.to(self.device)

        x_pls = F.relu(self.conv4(x_pls))
        x_pls = F.relu(self.conv4(x_pls))
        x_pls = F.relu(self.conv4(x_pls))
        # print("x_pls.size() = ", x_pls.size())

        ## batch_flattened_length = x_pls.shape[1] * x_pls.shape[2] * x_pls.shape[3]
        ## x = torch.empty(batch_size, batch_flattened_length)
        x = torch.flatten(x_pls, start_dim=1)
        # print("x.size() = ", x.size())

        x = F.relu(self.fc2(x))
        # print("fc2 -> x.size() = ", x.size())
        x = F.relu(self.fc3(x))
        # print("fc3 -> x.size() = ", x.size())
        adv = self.fc4_ea(x)
        val = self.fc4_ev(x)
        ## adv = torch.unsqueeze(adv, 0)
        # print("adv.size() = ", adv.size())
        ## val = torch.unsqueeze(val, 0)
        # print("val.size() = ", val.size())
        adv = adv.to('cpu')
        val = val.to('cpu')

        output = adv + val - adv.mean(1, keepdim=True).expand(-1, adv.size(1))
        # output = output.to(device)
        output = output.to(self.device)
        # print("output.size()", output.size())

        return output


class DDQN(Policy):
    def __init__(self):
        super().__init__()
        self.name = 'DDQN'
        self.trainable = True
        self.multiagent_training = None
        self.kinematics = None
        self.epsilon = None
        self.gamma = None
        self.sampling = None
        self.speed_samples = None
        self.rotation_samples = None
        self.query_env = None
        self.action_space = None
        self.speeds = None
        self.rotations = None
        self.action_values = None
        self.with_om = None
        self.cell_num = None
        self.cell_size = None
        self.om_channel_size = None
        self.self_state_dim = 6
        self.human_state_dim = 7
        self.joint_state_dim = self.self_state_dim + self.human_state_dim

    def configure(self, config):
        self.model = Network(1, 28)
        logging.info('Policy: DDQN for flow field based motion planner')

    def set_device(self, device):
        self.device = device
        self.model.to(device)

    def predict(self, state_m, state_g, state_b):
        output = self.model(state_m, state_g, state_b)

        return max_action






