# This is a script to generate URDF models for ROS simulation in rviz

class File:
	def __init__(self, *objects, filename='output.urdf.xacro'):
		self.objects = objects
		print(self.objects)
		self.xml = []
		self.filename = filename

	def __complete_xml(self):
		self.xml.insert(0, '<?xml version="1.0" ?>')
		self.xml.insert(0, '<robot>')
		self.xml.insert(len(self.xml), '</robot>')
		return self.xml

	def create_xml(self):
		with open("output.urdf.xacro", "w") as f:
			for key, val in enumerate(self.objects):
				print('TEST')
				# print(val.element)
				f.write(val.element)

		f.close()

class Property:
	def __init__(self, name, value):
		self.name = name
		self.value = value
		print(self.value)
		print(self.name)
		self.element = '<xacro:property name="radius" value="0.56"/>'

class Link:
	def __init__(self, rgba):
		self.rgba = rgba
		self.element = """\n
			<link name="test1">
   				<visual>
     				<geometry>
       					<box size="0.6 0.3 0.2"/>
     				</geometry>
     				<material name="green">
      					<color rgba="0.1 0.3 0.2"/>
     				</material>
   				</visual>
 			</link>
			"""

xml = File(
		Property('radius', '0.56'),
		Link('0.5 0.7 0.1 1')
	)

xml.create_xml()

filename = 'v3.output.urdf.xacro'

xacro_prop = """<xacro:property name="radius" value="0.56"/>"""

link = """\n
<link name="test1">
   <visual>
     <geometry>
       <box size="0.6 0.3 0.2"/>
     </geometry>
     <material name="green">
       <color rgba="%s"/>
     </material>
   </visual>
 </link>
"""

link2 = """\n
<link name="test2">
   <visual>
     <geometry>
       <box size="0.6 0.3 0.2"/>
     </geometry>
     <material name="green">
       <color rgba="%s"/>
     </material>
   </visual>
 </link>
"""

# reparsed = minidom.parseString(document)
		# print()

rgba = '0.5 0.5 1 0.5'

with open(filename, "w") as f:
	# f.write(reparsed.toprettyxml(indent='  '))
	f.write(xacro_prop)
	f.write(link % rgba)
	f.write(link2 % rgba)

f.close()

