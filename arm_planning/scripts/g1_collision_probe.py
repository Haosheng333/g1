#!/usr/bin/env python3
"""Report G1 arm/body collision pairs at chosen joint configurations.

This is a diagnostic step before static-mask generation. It deliberately reports
all candidate pairs so mounting contacts can be identified and exempted safely.
"""
from __future__ import annotations
from pathlib import Path
import argparse, json, math
import numpy as np
import trimesh
import fcl
import yaml


def rx(a):
    c,s=np.cos(a),np.sin(a); return np.array([[1,0,0],[0,c,-s],[0,s,c]],float)
def ry(a):
    c,s=np.cos(a),np.sin(a); return np.array([[c,0,s],[0,1,0],[-s,0,c]],float)
def rz(a):
    c,s=np.cos(a),np.sin(a); return np.array([[c,-s,0],[s,c,0],[0,0,1]],float)
def rpy_matrix(rpy):
    r,p,y=rpy; return rz(y)@ry(p)@rx(r)
def transform(xyz=(0,0,0),rpy=(0,0,0)):
    T=np.eye(4); T[:3,:3]=rpy_matrix(rpy); T[:3,3]=xyz; return T
def joint_rotation(axis,angle):
    axis=np.asarray(axis,float); axis/=np.linalg.norm(axis); x,y,z=axis; c=np.cos(angle); s=np.sin(angle); v=1-c
    R=np.array([[x*x*v+c,x*y*v-z*s,x*z*v+y*s],[y*x*v+z*s,y*y*v+c,y*z*v-x*s],[z*x*v-y*s,z*y*v+x*s,z*z*v+c]])
    T=np.eye(4); T[:3,:3]=R; return T


def fcl_transform(T):
    return fcl.Transform(np.asarray(T[:3,:3],dtype=np.float64),np.asarray(T[:3,3],dtype=np.float64))


def mesh_geometry(path):
    mesh=trimesh.load(path,force='mesh',process=False)
    if isinstance(mesh,trimesh.Scene): mesh=trimesh.util.concatenate(tuple(mesh.geometry.values()))
    model=fcl.BVHModel(); model.beginModel(len(mesh.vertices),len(mesh.faces)); model.addSubModel(np.asarray(mesh.vertices,dtype=np.float64),np.asarray(mesh.faces,dtype=np.int32)); model.endModel(); return model


def collide(obj_a,obj_b):
    request=fcl.CollisionRequest(num_max_contacts=1,enable_contact=True); result=fcl.CollisionResult(); count=fcl.collide(obj_a,obj_b,request,result)
    depth=None
    if count and result.contacts: depth=float(max(contact.penetration_depth for contact in result.contacts))
    return bool(count),depth


class G1CollisionProbe:
    def __init__(self,project_root,arm,config):
        self.root=Path(project_root); self.arm=arm; self.cfg=config; self.sign=1 if arm=='left' else -1
        meshes=self.root/'meshes'
        # Static body transforms expressed in torso_link.
        body_specs={
            'torso':('torso_link.STL',np.eye(4)),
            'head':('head_link.STL',transform((0.0039635,0,-0.054))),
            'pelvis':('pelvis.STL',transform((0.0039635,0,-0.054))),
            'waist_yaw':('waist_yaw_link.STL',transform((0.0039635,0,-0.054))),
            'waist_support':('waist_support_link.STL',transform((0.0039635,0,-0.054))),
        }
        self.body={name:fcl.CollisionObject(mesh_geometry(meshes/file),fcl_transform(T)) for name,(file,T) in body_specs.items()}

        prefix='left' if arm=='left' else 'right'
        # URDF collision primitives for shoulder pitch/roll.
        pitch_geom=fcl.Cylinder(0.03,0.05)
        roll_geom=fcl.Cylinder(0.03,0.03)
        self.arm_objects={
            'shoulder_pitch':fcl.CollisionObject(pitch_geom),
            'shoulder_roll':fcl.CollisionObject(roll_geom),
            'shoulder_yaw':fcl.CollisionObject(mesh_geometry(meshes/f'{prefix}_shoulder_yaw_link.STL')),
            'elbow':fcl.CollisionObject(mesh_geometry(meshes/f'{prefix}_elbow_link.STL')),
            'hand':fcl.CollisionObject(mesh_geometry(meshes/f'{prefix}_wrist_roll_rubber_hand.STL')),
        }
        sy=1.0 if arm=='left' else -1.0
        self.local_collision={
            'shoulder_pitch':transform((0,sy*0.04,-0.01),(0,math.pi/2,0)),
            'shoulder_roll':transform((-0.004,sy*0.006,-0.053)),
            'shoulder_yaw':np.eye(4),'elbow':np.eye(4),'hand':np.eye(4),
        }

    def link_transforms(self,q):
        q=np.asarray(q,float); a=self.cfg[self.arm]; T=np.eye(4); names=['shoulder_pitch','shoulder_roll','shoulder_yaw','elbow','hand']; output={}
        if q.shape == (4,): q5=np.r_[q,0.0]
        elif q.shape == (5,): q5=q
        else: raise ValueError('Expected four or five joint values')
        for i,name in enumerate(names):
            T=T@transform(a['origins_xyz'][i],a['origins_rpy'][i])@joint_rotation(a['axes'][i],q5[i])
            output[name]=T
        return output

    def allowance(self,first,second):
        collision_cfg=self.cfg.get('collision_checking',{})
        default=float(collision_cfg.get('default_penetration_allowance_m',0.0))
        return float(collision_cfg.get('penetration_allowance_m',{}).get(f'{first}__{second}',default))

    def report(self,q):
        links=self.link_transforms(q)
        for name,obj in self.arm_objects.items():
            obj.setTransform(fcl_transform(links[name]@self.local_collision[name]))
        results=[]
        def add(first,second,depth):
            allowance=self.allowance(first,second)
            blocking=(depth is None) or (depth>allowance)
            results.append({'pair':[first,second],'penetration_depth_m':depth,'allowance_m':allowance,'blocking':blocking})
        for moving,mobj in self.arm_objects.items():
            for fixed,bobj in self.body.items():
                hit,depth=collide(mobj,bobj)
                if hit: add(moving,fixed,depth)
        self_pairs=[tuple(v) for v in self.cfg.get('collision_checking',{}).get('self_collision_pairs',[])]
        for first,second in self_pairs:
            hit,depth=collide(self.arm_objects[first],self.arm_objects[second])
            if hit: add(first,second,depth)
        return results

    def is_blocked(self,q):
        return any(item['blocking'] for item in self.report(q))


def main():
    p=argparse.ArgumentParser(); p.add_argument('--arm',choices=['left','right'],required=True); p.add_argument('--joints',nargs=4,type=float,default=[0,0,0,0]); p.add_argument('--config',type=Path,default=Path('config/arm_kinematics.yaml')); p.add_argument('--project-root',type=Path,default=Path('.')); a=p.parse_args()
    cfg=yaml.safe_load(a.config.read_text(encoding='utf-8')); probe=G1CollisionProbe(a.project_root,a.arm,cfg); pairs=probe.report(a.joints)
    print(json.dumps({'arm':a.arm,'joints_rad':a.joints,'collision_count':len(pairs),'collisions':pairs},indent=2))

if __name__=='__main__': main()
