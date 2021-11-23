import os
import numpy as np
import random
import xml.etree.ElementTree as elemTree

def changer(xml_path, same_color=False):
	tree = elemTree.parse(xml_path)
	worldbody = tree.find('./worldbody')

	leg_num = [0,1,2,3]
	leg_part_num = [random.randint(0,2),random.randint(0,2),random.randint(0,2),random.randint(0,2)]
	#leg_part_num = [random.randint(0,1),random.randint(0,1),random.randint(0,1),random.randint(0,1)]
	while np.sum(leg_part_num) == 0:
		leg_num = [0,1,2,3]
		leg_part_num = [random.randint(0,2),random.randint(0,2),random.randint(0,2),random.randint(0,2)]

	leg_num = [0, 1, 2, 3]
	leg_part_num = [0, 1, 0, 0]
	if same_color:
		initial_color = '%f %f %f 1'%(np.random.random_sample(),np.random.random_sample(),np.random.random_sample())

	torso_size = 0.02#0.0225 + 0.0025*np.random.random_sample()
	torso_type = "sphere"

	"""for body in worldbody.iter('body'):
		if 'name' in body.attrib.keys():
			if body.attrib['name'] == str('torso'):
				body.attrib['pos'] = '%f %f 0.10'%(0.1*(np.random.random_sample()-0.5), 0.1*(np.random.random_sample()-0.5))"""
	
	for geom in worldbody.iter('geom'):
		if 'name' in geom.attrib.keys():
			if geom.attrib['name'] == 'torso_geom':
				if torso_type == "sphere":
					geom.attrib['type'] = "sphere"
					geom.attrib['size'] = str(torso_size)
				elif torso_type == "capsule":
					#torso_size = 0.01 + 0.005*np.random.random_sample()
					#torso_length = (0.04 + 0.06*np.random.random_sample())
					torso_size = 0.02
					torso_length = 0.15
					geom.attrib['type'] = "capsule"
					geom.attrib['fromto'] = '%f 0 0 %f 0 0'%(-torso_length, torso_length)
					geom.attrib['size'] = '%f %f'%(torso_size, torso_length)
				if same_color:
					geom.attrib['rgba'] = initial_color
				else:
					geom.attrib['rgba'] = '%f %f %f 1'%(np.random.random_sample(),np.random.random_sample(),np.random.random_sample())

	actuator = tree.find('./actuator')

	for body in worldbody.iter('body'):
		if 'name' in body.attrib.keys():
			if body.attrib['name'] == str('torso'):
				robot_base = body
				break                                                                                                                                                                                                                                         

	#elemTree.SubElement(robot_base, 'joint', axis='1 0 0', name='base_joint', pos='0 0 0', type='free')

	graph = []
	tracker_idx = 0
	joint_idx = 0
	geom_idx = 0
	body_idx = 0
	torso_tracker_idx = []

	for leg_idx in leg_num:
		if leg_part_num[leg_idx] == 0:
			continue

		body = elemTree.SubElement(robot_base, 'body', name='ant_body_%d'%(body_idx), pos='0 0 0')

		body_type = torso_type
		body_idx = body_idx + 1
		if body_type == "sphere":
			if leg_idx == 0:
				body.attrib['pos'] = '%f %f 0'%(torso_size, 0)
			elif leg_idx == 1:
				body.attrib['pos'] = '%f %f 0'%(0, torso_size + 0.022)
			elif leg_idx == 2:
				body.attrib['pos'] = '%f %f 0'%(-torso_size, 0)
			elif leg_idx == 3:
				body.attrib['pos'] = '%f %f 0'%(0, -torso_size)
		elif body_type == "capsule":
			if leg_idx == 0:
				body.attrib['pos'] = '%f %f 0'%(torso_length, 0)
			elif leg_idx == 1:
				body.attrib['pos'] = '%f %f 0'%(0, torso_size + 0.025)
			elif leg_idx == 2:
				body.attrib['pos'] = '%f %f 0'%(-torso_length, 0)
			elif leg_idx == 3:
				body.attrib['pos'] = '%f %f 0'%(0, -torso_size)

		sub_graph = [tracker_idx]
		torso_tracker_idx.append(tracker_idx)
		elemTree.SubElement(body, 'body', name='tracker_%d'%(tracker_idx), pos='0 0 0')
		tracker_idx = tracker_idx + 1

		for _ in range(0, leg_part_num[leg_idx]):
			sub_graph.append(tracker_idx)
			graph.append(sub_graph)
			sub_graph = [tracker_idx]
			body, joint_idx, geom_idx, body_idx, tracker_idx, body_type = add_geom(body, actuator, leg_idx, joint_idx, geom_idx, body_idx, tracker_idx, body_type, same_color=same_color)

	for i in range(len(torso_tracker_idx)):
		for j in range(i+1, len(torso_tracker_idx)):
			graph.append([torso_tracker_idx[i], torso_tracker_idx[j]])

	return tree, tracker_idx, graph

def add_geom(body, actuator, leg_idx, joint_idx, geom_idx, body_idx, tracker_idx, prev_body_type, same_color=False):

	joint_type = random.randint(0, 1)
	joint_type = 0
	if joint_type == 0:	
		elemTree.SubElement(body,'joint',axis='0 0 1',name='joint_%d'%(joint_idx),pos='0.0 -0.02 0',range='-1.0 1.0',type='hinge',limited='true')
		#elemTree.SubElement(actuator,'motor',ctrllimited='true',ctrlrange='-1.0 1.0',joint='joint_%d'%(joint_idx),gear='1')
		joint_idx = joint_idx + 1
	elif joint_type == 1:
		elemTree.SubElement(body,'joint',axis='0 1 0',name='joint_%d'%(joint_idx),pos='0 0 0',range='-30 30',type='hinge',limited='true')
		#elemTree.SubElement(actuator,'motor',ctrllimited='true',ctrlrange='-1.0 1.0',joint='joint_%d'%(joint_idx),gear='150')
		joint_idx = joint_idx + 1
	elif joint_type == 2:
		elemTree.SubElement(body,'joint',axis='1 0 0',name='joint_%d'%(joint_idx),pos='0 0 0',range='-30 30',type='hinge',limited='true')
		#elemTree.SubElement(actuator,'motor',ctrllimited='true',ctrlrange='-1.0 1.0',joint='joint_%d'%(joint_idx),gear='150')
		joint_idx = joint_idx + 1
	elif joint_type == 3:
		elemTree.SubElement(body,'joint',axis='1 0 0',name='joint_%d'%(joint_idx),pos='0 0 0',range='-0.1 0.1', type='slide',limited='true')
		#elemTree.SubElement(actuator,'motor',ctrllimited='true',ctrlrange='-1.0 1.0',joint='joint_%d'%(joint_idx),gear='150')
		joint_idx = joint_idx + 1

	if prev_body_type == 'capsule':
		body_type = random.sample(['sphere', 'capsule'], 1)[0]
	else:
		body_type = 'capsule'
	body_type = 'capsule'

	if body_type == 'capsule':
		#leg_length = (0.04 + 0.06*np.random.random_sample())
		#leg_size = 0.01 + 0.005*np.random.random_sample()
		leg_length = 0.1
		leg_size = 0.02
		if same_color:
			leg_color = initial_color
		else:
			leg_color = '%f %f %f 1'%(np.random.random_sample(),np.random.random_sample(),np.random.random_sample())

		if leg_idx == 0:
			pos = '%f %f 0'%(leg_length, 0)
		elif leg_idx == 1:
			pos = '%f %f 0'%(0, leg_length)
		elif leg_idx == 2:
			pos = '%f %f 0'%(-leg_length, 0)
		elif leg_idx == 3:
			pos = '%f %f 0'%(0, -leg_length)

		elemTree.SubElement(body, 'geom', fromto='0 0 0 '+pos, rgba=leg_color, name='geom_%d'%(geom_idx), size='%f %f'%(leg_size, leg_length/2), type='capsule', density='1')

		geom_idx = geom_idx + 1

		body = elemTree.SubElement(body, 'body', name='ant_body_%d'%(body_idx), pos=pos)
		body_idx = body_idx + 1
		elemTree.SubElement(body, 'body', name='tracker_%d'%(tracker_idx), pos='0 0 0')
		tracker_idx = tracker_idx + 1

	elif body_type == 'sphere':
		leg_length = 0.0225 + 0.0025*np.random.random_sample()
		if same_color:
			leg_color = initial_color
		else:
			leg_color = '%f %f %f 1'%(np.random.random_sample(),np.random.random_sample(),np.random.random_sample())
		if leg_idx == 0:
			pos = '%f %f 0'%(leg_length, 0)
		elif leg_idx == 1:
			pos = '%f %f 0'%(0, leg_length)
		elif leg_idx == 2:
			pos = '%f %f 0'%(-leg_length, 0)
		elif leg_idx == 3:
			pos = '%f %f 0'%(0, -leg_length)

		elemTree.SubElement(body, 'geom', pos=pos, rgba=leg_color, name='geom_%d'%(geom_idx), size='%f'%(leg_length), type='sphere')
		geom_idx = geom_idx + 1


		if leg_idx == 0:
			pos = '%f %f 0'%(2.0*leg_length, 0)
		elif leg_idx == 1:
			pos = '%f %f 0'%(0, 2.0*leg_length)
		elif leg_idx == 2:
			pos = '%f %f 0'%(-2.0*leg_length, 0)
		elif leg_idx == 3:
			pos = '%f %f 0'%(0, -2.0*leg_length)
		body = elemTree.SubElement(body, 'body', name='ant_body_%d'%(body_idx), pos=pos)
		body_idx = body_idx + 1
		elemTree.SubElement(body, 'body', name='tracker_%d'%(tracker_idx), pos='0 0 0')
		tracker_idx = tracker_idx + 1

	return body, joint_idx, geom_idx, body_idx, tracker_idx, body_type