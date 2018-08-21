#!/usr/bin/env python2
from lark import Lark, Transformer
import time
ROS=True
if ROS:
    import ros_mgr

FRAME_NAMES = {"map_frame" : "map", "bot_frame" : "bot"}

class TF(Transformer):
    def point(self, items):
#        print("transforming point: " + str(items))
        if len(items) > 2:
            return (float(items[0].value), float(items[1].value), float(items[2].value))
        return (float(items[0].value), float(items[1].value), 0.0) #default zaw to 0
#    def path(self, items):
#        print(items)
#        return items
    def frame_map(self,items):
        return "map"
    def frame_bot(self, items):
        return "base_footprint"
    def frame_odom(self, items):
        return "odom"
    def float(self, items):
        if len(items)==2:
            items = items[1:]
            items[0].value = -float(items[0].value)
        return items[0]
    def FLOAT(self, items):
        items[0].value = float(items[0].value)
        return items
    def INT(sel, items):
        items[0].value=int(items[0].value)
        return items
    def grid(self, items):
        ret = []
        for i in items:
            ret.append(int(i.value))
        return ret

def run(inst):
    print(inst)
    if inst.data == 'name':
        print("Executing plan: "+str(inst.children[0]))
        return
    if inst.data == 'restrict':
        print("setting nav restrictions")
        print("Setting nav restriction map resolution to "+ str(inst.children[0].children[0]))
        res = float(inst.children[0].children[0])
        if isinstance(inst.children[1], list):
            print("using grid for nav restrictions:" +str(inst.children[1]))
            grid = inst.children[1]
        else:
            print("reading nav restrictions from file: " + str(inst.children[1]))
            with open(inst.children[1]) as f:
                grid = eval(f.read())
        if ROS:
            ros_mgr.restrict(res, grid)
    if inst.data == 'sleep':
        print("sleeping for " + str(inst.children[1]) + "s")
        time.sleep(float(inst.children[1]))
        return
    if inst.data == 'speed':
        print("Setting platform speed to: " + str(inst.children[0].value) + str(inst.children[1].value) + str(inst.children[2].value))
        x = float(inst.children[0].value)
        y = float(inst.children[1].value)
        th = float(inst.children[2].value)
        if ROS:
            ros_mgr.setSpeed(x, y, th)
        return
    if inst.data == 'stop':
        print("stopping")
        if ROS:
            ros_mgr.setSpeed(0,0,0)
        return
    if inst.data == 'move':
        print("moving to point " + str(inst.children[0]) + str(inst.children[1]))
        x = float(inst.children[0].value)
        y = float(inst.children[1].value)
        if len(inst.children) > 2:
            frame = inst.children[2]
            print("move: using reference frame: " + str(inst.children[2]))
        else:
            print("move: using default reference frame map")
            frame = "map"
        if ROS:
            ros_mgr.moveTo(x, y, 0, frame)
            ros_mgr.wfc()
        return
    if inst.data == 'rot':
        print("rotating platform to yaw angle: " + inst.children[0])
        yaw = float(inst.children[0])
        if len(inst.children) > 1:
            frame = inst.children[1]
            print("rot: using reference fame " + str(inst.children[1]))
        else:
            frame = "map"
            print("rot: using default reference frame map")
        if ROS:
            ros_mgr.rotate(yaw, frame)
            ros_mgr.wfc()
        return
    if inst.data == 'trajectory':
        print("following trajectory using direct_move")
        if isinstance(inst.children[-1], str):
            waypoints = inst.children[:-1]
            print("waypoints: " +str(waypoints))
            frame = inst.children[-1]
            print("trajectory: using reference frame " + str(inst.children[-1]))
        else:
            waypoints = inst.children
            frame = "map"
            print("waypoints: " + str(waypoints))
            print("trajectory: using default reference frame map")
        if ROS:
            ros_mgr.trajectory(waypoints, frame)
            ros_mgr.wfc()
        return
    if inst.data=='path':
        tgt = inst.children[0]
        print("planning path to: " + str(tgt))
        if ROS:
            ros_mgr.pathplan(tgt)
            ros_mgr.wfc()
        return
    if inst.data == 'path_t':
        print(inst.children)
        points = inst.children
        print("planning path via: " + str(points))
        if ROS:
            ros_mgr.trajectoryplan(points)
            ros_mgr.wfc()
        return

plan = open("plan.dsl").read()
#print(plan)
parser = Lark(open("dsl.ebnf"), parser="lalr")
tree=parser.parse(plan)
print(tree)
tree = TF().transform(tree)
print(tree)
if ROS:
    ros_mgr.init()
    time.sleep(2)
for inst in tree.children:
    #print(inst)
    run(inst)
time.sleep(0.5)

