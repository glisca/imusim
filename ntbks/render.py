# from imusim.all import *

from imusim.io.bvh import CM_TO_M_CONVERSION
from imusim.io.bvh import loadBVHFile

from imusim.trajectories.rigid_body import SplinedBodyModel

from imusim.visualisation.rendering import BodyModelRenderer
from imusim.visualisation.rendering import VelocityVectorRenderer
from imusim.visualisation.rendering import AccelerationVectorRenderer
from imusim.visualisation.rendering import InteractiveAnimation

# model = loadBVHFile("/home/lisca/code/imusim/imusim/tests/system/walk.bvh", CM_TO_M_CONVERSION)
# model = loadBVHFile("/home/lisca/data/mocap/01_01.bvh", CM_TO_M_CONVERSION)
model = loadBVHFile("/home/lisca/data/total/s1/acting1_BlenderZXY_YmZ.bvh", CM_TO_M_CONVERSION)

splinedModel = SplinedBodyModel(model)

start = model.startTime
end = model.endTime

renderer = BodyModelRenderer(model)
# animation = InteractiveAnimation(start, end, renderer)

# trajectory velocity renderer
# walk_rndrr = VelocityVectorRenderer(splinedModel.getJoint('rfoot'), opacity=0.5, color=(1,0,0))
# animation = InteractiveAnimation(start, end, renderer, walk_rndrr)

# amass_rndrr = VelocityVectorRenderer(splinedModel.getJoint('rFoot'), opacity=0.5, color=(1,0,0))
# animation = InteractiveAnimation(start, end, renderer, amass_rndrr)

total_vlcty_rndrr = VelocityVectorRenderer(splinedModel.getJoint('LeftHand'), opacity=0.5, color=(1,0,0))
total_acclrtn_rndrr = AccelerationVectorRenderer(splinedModel.getJoint('LeftHand'), opacity=0.5, color=(0,0,1))
animation = InteractiveAnimation(start, end, renderer, total_vlcty_rndrr, total_acclrtn_rndrr)

# sampledRenderer = BodyModelRenderer(model, opacity=0.5, color=(1,0,0))
# splinedRenderer = BodyModelRenderer(splinedModel, opacity=0.5, color=(0,0,1))

# animation = InteractiveAnimation(start, end, sampledRenderer, splinedRenderer)