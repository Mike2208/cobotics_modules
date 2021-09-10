import torch
import torch.nn as nn
import torch.nn.functional as F
import os
import matplotlib.pyplot as plt
from src.prednet_segmentation.ConvLSTMCell import ConvLSTMCell
from src.prednet_segmentation.hConvGRUCell import hConvGRUCell

class PredNet(nn.Module):
    
  def __init__(self,
    model_name, device,
    A_channels, R_channels, R_layers, S_layers,
    filter_sizes, dropout_rates, n_classes,
    do_time_aligned, do_prediction, do_segmentation):
      
    # Initialize parameters
    super(PredNet, self).__init__()
    self.a_channels = A_channels
    self.r_channels = R_channels + (0,)  # for convenience
    self.n_layers = len(R_channels)
    self.r_layers = R_layers
    self.s_layers = S_layers
    self.filter_sizes = filter_sizes
    self.filter_pads = [(s-1)//2 for s in self.filter_sizes]
    self.dropout_rates = dropout_rates
    self.do_time_aligned = do_time_aligned
    self.do_prediction = do_prediction
    self.do_segmentation = do_segmentation
    self.n_classes = n_classes
    self.model_name = model_name
    self.device = device
    p = f'./ckpt/{model_name}/'
    if not os.path.exists(p):
      os.mkdir(p)

    # Upsampling (for the top down pass between layers)
    for l in range(1, self.n_layers):
      # upsample = nn.upsample(scale_factor=2)
      upsample = nn.ConvTranspose2d(
        in_channels=self.r_channels[l],
        out_channels=self.r_channels[l],
        kernel_size=self.filter_sizes[l], stride=2, padding=self.filter_pads[l], output_padding=1)
      setattr(self, 'upsample{}'.format(l), upsample)
    
    # Initialize representation part for all layers ("A[l] + R[l+1] -> R[l]")
    for l in range(self.n_layers):
      input_channels = self.a_channels[l]*2 + self.r_channels[l+1]
      output_channels = self.r_channels[l]
      if self.r_layers[l] == 'conv':
        cell = nn.Sequential(
          nn.Conv2d(
            in_channels=input_channels,
            out_channels=output_channels,
            kernel_size=self.filter_sizes[l],
            padding=self.filter_pads[l]),
          nn.ReLU())
      elif self.r_layers[l] == 'lstm':
        cell = ConvLSTMCell(
          input_channels,
          output_channels,
          self.filter_sizes[l])
      elif self.r_layers[l] == 'hgru':
        cell = hConvGRUCell(
          input_channels,
          output_channels,
          self.filter_sizes[l])
      setattr(self, 'cell{}'.format(l), cell)

    # Initialize prediction part for all layers ("R[l] -> A_hat[l]")
    if self.do_prediction:
      for l in range(self.n_layers):
        conv_pred = nn.Sequential(
          nn.Conv2d(
            in_channels=self.r_channels[l],
            out_channels=self.a_channels[l],
            kernel_size=self.filter_sizes[l],
            padding=self.filter_pads[l]),
          nn.ReLU())
        if l == 0:
          conv_pred.add_module('satlu', SatLU())
        setattr(self, 'conv_pred{}'.format(l), conv_pred)

    # Dropout layers just before input
    for l in range(self.n_layers):
      dropout_A = nn.Dropout(p=dropout_rates[l])
      setattr(self, 'dropout_A{}'.format(l), dropout_A)

    # Initialize input part for all layers ("A[l]-P[l] -> A[l+1]")
    for l in range(self.n_layers - 1):
      update_A = nn.Sequential(
        nn.Conv2d(
          in_channels=self.a_channels[l]*2,
          out_channels=self.a_channels[l+1],
          kernel_size=self.filter_sizes[l],
          padding=self.filter_pads[l]),
        nn.MaxPool2d(kernel_size=2, stride=2))
      setattr(self, 'update_A{}'.format(l), update_A)

    # Initialize segmentation part
    if self.do_segmentation:
      for l in self.s_layers:
        upsample_list = []
        for n in range(l):
          upsample_list.append(
            nn.Sequential(
              nn.ConvTranspose2d(
                in_channels=self.r_channels[l],
                out_channels=self.r_channels[l],
                kernel_size=3, stride=2, padding=1, output_padding=1),
              nn.InstanceNorm2d(self.r_channels[l])))
        upsample_segm = nn.Sequential(*upsample_list)
        setattr(self, 'upsample_segm{}'.format(l), upsample_segm)
      input_channels = sum([self.r_channels[l] for l in self.s_layers])
      self.conv_segm = nn.Sequential(
        nn.Conv2d(
          in_channels=input_channels,
          out_channels=self.n_classes,
          kernel_size=3,
          padding=1),
        nn.Sigmoid())

    # Put the model on the correct device
    self.to(device)

  def forward(self, input_):  # input dims = (batch_size, n_channels, im_w, im_h, n_frames)

    # Initialize states
    TA = self.do_time_aligned  # for convenience
    batch_size, n_channels, w, h, n_frames = input_.size()
    P_seq = torch.zeros_like(input_)  # [batch_size, n_channels, w, h, n_frames]
    S_seq = torch.zeros(batch_size, self.n_classes, w, h, n_frames).cuda()
    E_seq, R_seq, L_seq = [], [], []
    for l in range(self.n_layers):
      E_seq.append([None]*(TA+n_frames))  # [n_layers, n_frames+1][batch_sizes, a_channels, w, h]
      R_seq.append([None]*(TA+n_frames))  # [n_layers, n_frames+1][batch_sizes, r_channels, w, h]
      L_seq.append([None]*(TA+n_frames))  # only used if lstm in one of the r_layers
      for t in range(n_frames):
        if t == 0 or not self.do_time_aligned:
          E_seq[l][t] = torch.zeros(batch_size, 2*self.a_channels[l], w, h).cuda()
          R_seq[l][t] = torch.zeros(batch_size, 1*self.r_channels[l], w, h).cuda()
          if self.r_layers[l] == 'lstm':
            L_seq[l][t] = torch.zeros(batch_size, 1*self.r_channels[l], w, h).cuda()
      w = w//2
      h = h//2

    # Loop through the time-steps // may be time aligned is replace all '1' by '0'
    for t in range(TA, TA+n_frames):
      A = input_[..., t-TA]

      # Top-down pass
      for l in reversed(range(self.n_layers)):
        cell = getattr(self, 'cell{}'.format(l))
        R = R_seq[l][t-TA]
        E = E_seq[l][max(t-1, 0)]  # not 't-TA' on purpose
        if l != self.n_layers - 1:
          upsample = getattr(self, 'upsample{}'.format(l+1))
          E = torch.cat((E, upsample(R_seq[l+1][t-TA])), dim=1)
        if self.r_layers[l] == 'conv':
          R = cell(E)
        elif self.r_layers[l] == 'lstm':
          R, C = cell(E, R, L_seq[l][t-TA])
          L_seq[l][t] = C
        elif self.r_layers[l] == 'hgru':
          R = cell(E, R, t-TA)
        R_seq[l][t] = R

      # Bottom-up pass
      for l in range(self.n_layers):
        dropout_A = getattr(self, 'dropout_A{}'.format(l))
        A = dropout_A(A)
        if self.do_prediction:
          conv_pred = getattr(self, 'conv_pred{}'.format(l))
          A_hat = conv_pred(R_seq[l][t-TA])
          if l == 0:
            P_seq[..., t-TA] = A_hat
          pos = F.relu(A_hat - A)
          neg = F.relu(A - A_hat)
          E_seq[l][t] = torch.cat([pos, neg], 1)
        else:
          E_seq[l][t] = torch.cat([F.relu(A), torch.zeros_like(A, requires_grad=False)], 1)
        if l < self.n_layers - 1:
          update_A = getattr(self, 'update_A{}'.format(l))
          A = update_A(E_seq[l][t-TA])  # fed to E_seq[l+1]

      # Segmentation
      if self.do_segmentation:
        S_input = [None for l in self.s_layers]
        for i, l in enumerate(self.s_layers):
          upsample_segm = getattr(self, 'upsample_segm{}'.format(l))
          S_input[i] = upsample_segm(R_seq[l][t-TA])
        S_hat = self.conv_segm(torch.cat(S_input, dim=1))
        S_seq[..., t-TA] = S_hat

    return [E[TA:] for E in E_seq], [R[TA:] for R in R_seq], P_seq, S_seq


class SatLU(nn.Module):

  def __init__(self, lower=0.0, upper=1.0, inplace=False):
    super(SatLU, self).__init__()
    self.lower = lower
    self.upper = upper
    self.inplace = inplace

  def forward(self, x):
    return F.hardtanh(x, self.lower, self.upper, self.inplace)

if __name__ == "__main__":

  batch_size = 16
  n_channels = 3
  im_w, im_h, n_channels = (64, 64, 3)
  n_frames = 10
  device = 'cuda'

  torch.cuda.set_per_process_memory_fraction(0.4, 0)

  model = PredNet(
    model_name='prednet',
    device=device,
    A_channels=(n_channels, 16, 64),
    R_channels=(16, 64, 256),
    R_layers=('hgru', 'hgru', 'hgru'),
    S_layers=(1, 1, 1),
    filter_sizes=(3, 5, 7),
    dropout_rates=(0.1, 0.1, 0.1),
    n_classes=3,
    do_time_aligned=True,
    do_prediction=True,
    do_segmentation=True)

  input_ = torch.rand(
    batch_size, n_channels, im_w, im_h, n_frames)
  E_seq, R_seq, P_seq, S_seq = model(input_.to(device))
  for frame in range(n_frames):
    plt.subplot(1, 2, 1)
    plt.imshow(P_seq[0, ..., frame].permute(1, 2, 0).detach().cpu())
    plt.title(f'Frame {frame} prediction')
    plt.subplot(1, 2, 2)
    plt.imshow(S_seq[0, ..., frame].permute(1, 2, 0).detach().cpu())
    plt.title(f'Frame {frame} segmentation')
    plt.show()
