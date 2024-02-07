#!/usr/bin/env python3

import torch
from torch import nn   
import torch.nn.functional as F
from torch.distributions import Categorical

import numpy as np
    
class PPO():
  def __init__(self, n_sts: int = 4, n_act: int = 3, dim: int = 256, lr: float = 5e-4, gamma: float = 0.98, gae: float =0.95, eps: float=0.1):
    
    self.act = nn.Sequential(
    	nn.Linear(n_sts, dim),
    	nn.ReLU(),
    	nn.Linear(dim, n_act),
    	nn.Softmax()
    )
    
    self.cri = nn.Sequential(
    	nn.Linear(n_sts, dim),
    	nn.ReLU(),
    	nn.Linear(dim, 1)
    )
    
    self.act_opt = torch.optim.Adam(self.act.parameters(), lr=lr)
    self.cri_opt = torch.optim.Adam(self.cri.parameters(), lr=lr)
    
    self.gamma = gamma
    self.gae = gae
    self.eps = eps
    
    self.buffer = [[], [], [], [], [], []]

  def get_action(self, s):    
    prob = self.act(torch.tensor(np.array(s)).type(torch.float))
    m = Categorical(prob)
    a = m.sample().item()
    return a, prob[a].item()
    
  def update(self):
    if len(self.buffer[0]) < 1:
      return 0
      
    datas = self.get_sample()
    
    s, a, r, s_p, done, a_prob = [d for d in datas]
    r /= 10

    td_target = r + self.gamma * self.cri(s_p) * (done == False)
    advs = (td_target - self.cri(s))

    a_hats = []
    a_hat = 0
    for adv in advs.detach().numpy()[::-1]:
      a_hat = adv + self.gamma * self.gae * a_hat
      a_hats.append(a_hat)
        
    a_hats.reverse()
    a_hats = torch.tensor(a_hats)
      
    cri_loss = F.mse_loss(self.cri(s), td_target.detach())
    self.cri_opt.zero_grad()
    cri_loss.backward()
    self.cri_opt.step()
      
    pi = self.act(s)
    pi_a = pi.gather(1, a) / a_prob
    
    act_loss = - torch.min(pi_a * a_hats, torch.clamp(pi_a, 1 - self.eps, 1 + self.eps) * a_hats)
    act_loss = act_loss.mean()
      
    self.act_opt.zero_grad()
    act_loss.backward()
    self.act_opt.step()

    return act_loss
      
  def add_data(self, data):
    s, a, r, s_p, done, a_prob = data
    self.buffer[0].append(s)
    self.buffer[1].append([a])
    self.buffer[2].append([r])
    self.buffer[3].append(s_p)
    self.buffer[4].append([done])
    self.buffer[5].append([a_prob])
    
  def get_sample(self):
    
    torch_lst = []  
    for d in self.buffer:
      if isinstance(d[0][0], torch.Tensor):
       torch_lst.append(torch.tensor(d))
      else:
       torch_lst.append(torch.tensor(np.array(d), dtype=torch.int64 if isinstance(d[0][0], int) else torch.float))
       
    self.buffer = [[], [], [], [], [], []]

    return torch_lst
    
