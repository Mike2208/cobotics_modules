# Taken from: https://github.com/c-rbp/pathfinder_experiments/blob/main/models/hgrucleanSEG.py
# Paper: https://arxiv.org/abs/2010.15314
import torch
import torch.nn.functional as F
from torch import nn

class hConvGRUCell(nn.Module):

    def __init__(self, input_size, hidden_size, kernel_size, batchnorm=True, timesteps=20):
        super().__init__()

        self.padding = kernel_size//2
        self.input_size = input_size
        self.hidden_size = hidden_size
        self.timesteps = timesteps
        self.batchnorm = batchnorm

        self.u1_gate = nn.Conv2d(hidden_size, hidden_size, 1)
        self.u2_gate = nn.Conv2d(hidden_size, hidden_size, 1)
        self.w_gate_inh = nn.Parameter(torch.empty(hidden_size , hidden_size , kernel_size, kernel_size))
        self.w_gate_exc = nn.Parameter(torch.empty(hidden_size , hidden_size , kernel_size, kernel_size))
        if hidden_size != input_size:  # added by me
            self.w_input_dim = nn.Parameter(torch.empty(hidden_size , input_size , 1, 1))
            nn.init.orthogonal_(self.w_input_dim)
        else:
            self.w_input_dim = None
        self.alpha = nn.Parameter(torch.empty((hidden_size, 1, 1)))
        self.gamma = nn.Parameter(torch.empty((hidden_size, 1, 1)))
        self.kappa = nn.Parameter(torch.empty((hidden_size, 1, 1)))
        self.w = nn.Parameter(torch.empty((hidden_size, 1, 1)))
        self.mu= nn.Parameter(torch.empty((hidden_size, 1, 1)))
        if self.batchnorm:
            self.bn = nn.ModuleList([nn.BatchNorm2d(hidden_size, eps=1e-03)
            # self.bn = nn.ModuleList([nn.InstanceNorm2d(hidden_size, eps=1e-03)
                for t in range(4*self.timesteps)])
        else:
            self.n = nn.Parameter(torch.randn(4*self.timesteps, 1, 1))

        nn.init.orthogonal_(self.w_gate_inh)
        nn.init.orthogonal_(self.w_gate_exc)
        self.w_gate_inh.register_hook(lambda grad: (grad + torch.transpose(grad, 1, 0))*0.5)
        self.w_gate_exc.register_hook(lambda grad: (grad + torch.transpose(grad, 1, 0))*0.5)
        nn.init.orthogonal_(self.u1_gate.weight)
        nn.init.orthogonal_(self.u2_gate.weight)
        if self.batchnorm:
            for bn in self.bn:
                nn.init.constant_(bn.weight, 0.1)
        nn.init.constant_(self.alpha, 0.1)
        nn.init.constant_(self.gamma, 1.0)
        nn.init.constant_(self.kappa, 0.5)
        nn.init.constant_(self.w, 0.5)
        nn.init.constant_(self.mu, 1)
        nn.init.uniform_(self.u1_gate.bias.data, 1, 8.0 - 1)
        self.u1_gate.bias.data.log()
        self.u2_gate.bias.data =  -self.u1_gate.bias.data


    def forward(self, input_, prev_state2, t=0):

        # if t == 0:
        #     prev_state2 = torch.empty_like(input_)
        #     nn.init.xavier_normal_(prev_state2)
        if self.w_input_dim is not None:  # adapt input size (added by me)
            input_ = F.conv2d(input_, self.w_input_dim)

        if self.batchnorm:
            g1_t = torch.sigmoid(self.bn[t*4+0](self.u1_gate(prev_state2)))
            c1_t = self.bn[t*4+1](F.conv2d(prev_state2*g1_t, self.w_gate_inh, padding=self.padding))
            next_state1 = F.relu(input_ - F.relu(c1_t*(self.alpha*prev_state2 + self.mu)))
            g2_t = torch.sigmoid(self.bn[t*4+2](self.u2_gate(next_state1)))
            c2_t = self.bn[t*4+3](F.conv2d(next_state1, self.w_gate_exc, padding=self.padding))
            h2_t = F.relu(self.kappa*next_state1 + self.gamma*c2_t + self.w*next_state1*c2_t)          
            prev_state2 = (1 - g2_t)*prev_state2 + g2_t*h2_t

        else:
            g1_t = torch.sigmoid(self.u1_gate(prev_state2))
            c1_t = F.conv2d(prev_state2 * g1_t, self.w_gate_inh, padding=self.padding)
            next_state1 = torch.tanh(input_ - c1_t*(self.alpha*prev_state2 + self.mu))
            g2_t = torch.sigmoid(self.n[t*4+2]*(self.u2_gate(next_state1)))
            c2_t = F.conv2d(next_state1, self.w_gate_exc, padding=self.padding)
            h2_t = torch.tanh(self.kappa*(next_state1 + self.gamma*c2_t) + (self.w*(next_state1*(self.gamma*c2_t))))
            prev_state2 = self.n[t]*((1 - g2_t)*prev_state2 + g2_t*h2_t)

        return prev_state2