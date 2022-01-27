import os
import time
import pybullet as p
import pybullet_data
import random

p.connect(p.GUI)
# cameraDistance, cameraYaw, cameraPitch, "cameraTargetPosition
p.resetDebugVisualizerCamera(3, 90, -30, [0.0, -0.0, -0.0])
p.setTimeStep(1 / 240.)
# p.setAdditionalSearchPath(pybullet_data.getDataPath())
print(pybullet_data.getDataPath())
# load urdf data
models_path = '/root/ocrtoc_ws/src/pybullet_tests/models'

# Load table and plane
p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"))
p.loadURDF(os.path.join(pybullet_data.getDataPath(), "table/table.urdf"))

# load the randomly picked model
flags = p.URDF_USE_INERTIA_FROM_FILE
# get a model
print("!!")
print(os.path.isdir(models_path))
random_model_path=(os.path.join(models_path, random.choice(os.listdir(models_path)) ))
print(random_model_path)
random_model_path=os.path.join(random_model_path, 'collision.obj')
print(random_model_path)
world_path = '/root/ocrtoc_ws/src/pybullet_tests/1-1-1.world'
scene_object_ids = p.loadSDF(world_path)
# p.loadURDF(random_model_path, [0, 0, 0], flags=flags)

p.setGravity(0, 0, -9.8)

while 1:
    p.stepSimulation()
    time.sleep(1./240)