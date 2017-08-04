# This is a script to generate URDF models for ROS simulation in rviz

class File:
	def __init__(self, *objects, filename='output.urdf.xacro'):
		self.objects = objects
		self.xml = []
		self.filename = filename

	def __complete_xml(self):
		self.xml.insert(0, '<?xml version="1.0" ?>')
		self.xml.insert(0, '<robot>')
		self.xml.insert(len(self.xml), '</robot>')
		return self.xml

	def save(self):
		with open(self.filename, "w") as f:
			for key, val in enumerate(self.objects):
				f.write(val.element)

		print("-- Success --")
		f.close()

class Property:
	def __init__(self, name, value):
		self.name = name
		self.value = value
		self.element = '<xacro:property name="radius" value="0.56"/>'

class Link:
	def __init__(self, rgba):
		self.rgba = rgba
		self.element = '\n\n<link name="test1">\n\t<visual>\n\t\t<geometry>\n\t\t\t<box size="0.6 0.3 0.2"/>\n\t\t</geometry>\n\t\t<material name="green">\n\t\t\t<color rgba="0.1 0.3 0.2"/>\n\t\t</material>\n\t</visual>\n</link>'
			

xml = File(
		Property('radius', '0.56'),
		Link('0.5 0.7 0.1 1')
	)

xml.save()

