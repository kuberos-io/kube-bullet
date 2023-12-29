
from loguru import logger
import pybullet as p
import numpy as np


def create_pose_marker(position=np.array([0, 0, 0]),
					 orientation=np.array([0, 0, 0, 1]),
                     text="",
                     xColor=np.array([1, 0, 0]),
                     yColor=np.array([0, 1, 0]),
                     zColor=np.array([0, 0, 1]),
                     textColor=np.array([0, 0, 0]),
                     lineLength=0.2,
                     lineWidth=3,
                     textSize=1,
                     textOrientation=None,
                     lifeTime=0,
                     parentObjectUniqueId=-1,
                     parentLinkIndex=-1,
                     physicsClientId=0,
                     replaceItemUniqueIdList=[-1]*4):
	"""
    Create a pose marker to show the position and orientation or a link
    in space with 3 colored lines.
    """

	logger.info(f"Create coodirnate marker with text: {text}")
 
	pts = np.array([[0, 0, 0], [lineLength, 0, 0], [0, lineLength, 0], [0, 0, lineLength]])
	rotIdentity = np.array([0, 0, 0, 1])

	po, _ = p.multiplyTransforms(position, orientation, pts[0, :], rotIdentity)
	px, _ = p.multiplyTransforms(position, orientation, pts[1, :], rotIdentity)
	py, _ = p.multiplyTransforms(position, orientation, pts[2, :], rotIdentity)
	pz, _ = p.multiplyTransforms(position, orientation, pts[3, :], rotIdentity)

	# add lines
	id_x = p.addUserDebugLine(
		po, px, xColor, lineWidth, lifeTime,
	    parentObjectUniqueId,
     	parentLinkIndex,
    	replaceItemUniqueId=replaceItemUniqueIdList[0])
	
	id_y = p.addUserDebugLine(
		po, py, yColor, lineWidth, lifeTime,
		parentObjectUniqueId,
  		parentLinkIndex,
		replaceItemUniqueId=replaceItemUniqueIdList[1])

	id_z = p.addUserDebugLine(
     	po, pz, zColor, lineWidth, lifeTime,
		parentObjectUniqueId,
  		parentLinkIndex, 
    	replaceItemUniqueId=replaceItemUniqueIdList[2])

	# add label text
	if textOrientation is None:
		textOrientation = orientation
	id_t = p.addUserDebugText(
		text, pz, 
  		textColorRGB=textColor,
    	textSize=textSize,
		lifeTime=lifeTime,
		parentObjectUniqueId=parentObjectUniqueId,
		parentLinkIndex=parentLinkIndex,
		physicsClientId=physicsClientId,
		replaceItemUniqueId = replaceItemUniqueIdList[3])
	
	return id_x, id_y, id_z, id_t
