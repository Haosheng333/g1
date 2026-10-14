#!/usr/bin/env python3
"""Differentiable Unitree G1 five-joint FK implemented in PyTorch."""
from __future__ import annotations
import torch
from torch import nn


def _rpy_matrix(rpy, dtype=torch.float32):
    r, p, y = [torch.tensor(float(v), dtype=dtype) for v in rpy]
    cr, sr = torch.cos(r), torch.sin(r); cp, sp = torch.cos(p), torch.sin(p); cy, sy = torch.cos(y), torch.sin(y)
    return torch.stack((
        torch.stack((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr)),
        torch.stack((sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr)),
        torch.stack((-sp, cp*sr, cp*cr)),
    ))


def _axis_rotation(axis, angle):
    axis = torch.as_tensor(axis, dtype=angle.dtype, device=angle.device)
    axis = axis / torch.linalg.vector_norm(axis)
    x, y, z = axis.unbind(); c, s, v = torch.cos(angle), torch.sin(angle), 1.0-torch.cos(angle)
    row0 = torch.stack((x*x*v+c, x*y*v-z*s, x*z*v+y*s), dim=-1)
    row1 = torch.stack((y*x*v+z*s, y*y*v+c, y*z*v-x*s), dim=-1)
    row2 = torch.stack((z*x*v-y*s, z*y*v+x*s, z*z*v+c), dim=-1)
    return torch.stack((row0,row1,row2), dim=-2)


class TorchG1FK(nn.Module):
    """Maps joint tensors shaped (B,5) to poses [x,y,z,rx,rz]."""
    def __init__(self, arm: str, config: dict):
        super().__init__(); cfg=config[arm]
        self.register_buffer('origins_xyz', torch.tensor(cfg['origins_xyz'], dtype=torch.float32))
        self.register_buffer('origins_rpy_R', torch.stack([_rpy_matrix(v) for v in cfg['origins_rpy']]))
        self.register_buffer('axes', torch.tensor(cfg['axes'], dtype=torch.float32))
        self.register_buffer('tool_xyz', torch.tensor(cfg['grasp_xyz'], dtype=torch.float32))
        self.register_buffer('tool_R', _rpy_matrix(cfg['grasp_rpy']))

    def forward(self, q):
        if q.ndim != 2 or q.shape[1] != 5: raise ValueError(f'Expected (B,5), received {tuple(q.shape)}')
        b=len(q); R=torch.eye(3,device=q.device,dtype=q.dtype).expand(b,3,3).clone(); t=torch.zeros((b,3),device=q.device,dtype=q.dtype)
        for i in range(5):
            p0=self.origins_xyz[i].to(dtype=q.dtype); R0=self.origins_rpy_R[i].to(dtype=q.dtype)
            t=t+torch.einsum('bij,j->bi',R,p0)
            R=torch.matmul(R,R0)
            R=torch.matmul(R,_axis_rotation(self.axes[i],q[:,i]))
        t=t+torch.einsum('bij,j->bi',R,self.tool_xyz.to(dtype=q.dtype)); R=torch.matmul(R,self.tool_R.to(dtype=q.dtype))
        pitch=torch.atan2(-R[:,2,0],torch.hypot(R[:,0,0],R[:,1,0]))
        normal=torch.abs(torch.cos(pitch))>1e-7
        roll_normal=torch.atan2(R[:,2,1],R[:,2,2]); yaw_normal=torch.atan2(R[:,1,0],R[:,0,0])
        roll_lock=torch.atan2(-R[:,0,1],R[:,1,1]); zero=torch.zeros_like(roll_lock)
        roll=torch.where(normal,roll_normal,roll_lock); yaw=torch.where(normal,yaw_normal,zero)
        return torch.cat((t,roll[:,None],yaw[:,None]),dim=1)
