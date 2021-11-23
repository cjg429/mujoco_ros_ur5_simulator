import xml.etree.ElementTree as elemTree

def merge_robot_and_object(robot_xml, object_xml):

	robot_tree = elemTree.parse(robot_xml)
	object_tree = elemTree.parse(object_xml)

	robot_asset = robot_tree.find('./asset')
	object_asset = object_tree.find('./asset')
	for i in range(len(object_asset)):
		robot_asset.append(object_asset[i])

	robot_worldbody = robot_tree.find('./worldbody')
	object_worldbody = object_tree.find('./worldbody')
	for body in object_worldbody.iter('body'):
		if 'name' in body.attrib.keys():
			if body.attrib['name'] == str('torso'):
				object_base = body
	robot_worldbody.append(object_base)

	robot_actuator = robot_tree.find('./actuator')
	object_actuator = object_tree.find('./actuator')
	for i in range(len(object_actuator)):
		robot_actuator.append(object_actuator[i])
	num_joint = len(object_actuator)
	
	return robot_tree, num_joint

	#print(robot_worldbody)
	#robot_worldbody.
	#print(object_body)
	#print(object_body[0])
	#print(object_body["name"])
	#print(robot_worldbody, object_worldbody)