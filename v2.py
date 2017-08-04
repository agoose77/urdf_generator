from xml.dom import minidom
    # EXPERIMENT
class XML:
	"""
	XML class
	Creates a beautified xml file from urdf objects
	"""
	def __init__(self, filename, *objects):
		self.objects = objects
		# print("Objects", self.objects)
		# print(self.objects)
		self.filename = filename
		# print(self.filename)
		self.xml = []

	def dict2xml(self, d, name, root_node=None):
		"""
		Converts given dictionary to xml string
		Taken from https://gist.github.com/reimund/5435343
		small changes made to accomodate for link name, see link_root
		"""
		wrap          = False if None == root_node or isinstance(d, list) else True
		root          = 'link' if None == root_node else root_node
		root_singular = root[:-1] if 's' == root[-1] and None == root_node else root
		xml           = ''
		children      = []
		link_root     = root + ' name="' + name + '"'
		# print(d)
		# property_root = root

		if isinstance(d, dict):
			for key, value in dict.items(d):
				if isinstance(value, dict):
					children.append(self.dict2xml(value, name, key))
				elif isinstance(value, list):
					children.append(self.dict2xml(value, name, key))
				else:
					xml = xml + ' ' + key + '="' + str(value) + '"'
		else:
			for value in d:
				children.append(self.dict2xml(value, name, root_singular))

		end_tag = '>' if 0 < len(children) else '/>'
		# end_tag = '<xacro:property' if root == 'xacro:property' else '/>'


		if wrap or isinstance(d, dict):
			if root == 'link':
				xml = '<' + link_root + xml + end_tag
			elif root == 'xacro:property':
				end_tag = '<xacro:property' if 'xacro:property' in d.keys() else end_tag
				xml = xml + end_tag
			else:
				# print("HERE1")
				xml = '<' + root + xml + end_tag

		if 0 < len(children):
			# print("HERE3")
			for child in children:
				xml = xml + child

			if root == 'xacro:property':
				xml = xml + ''
			# if wrap or isinstance(d, dict):
			else:
				xml = xml + '</' + root + '>'

		# print(xml)

		return xml

	def create_xml(self):
		for ind, val in enumerate(self.objects):
			# print(val.dict2xml(val.link))
			if isinstance(val, Property):
				print("PROP")
			# print(type(val))
			self.xml.append(self.dict2xml(val.element, val.name, val.element_type))
			# self.xml.append('')
		# print(self.xml)

	def __complete_xml(self):
		self.xml.insert(0, "<robot> ")
		self.xml.insert(len(self.xml), " </robot>")
		return self.xml


	def save(self):
		# reparsed = minidom.parseString(' '.join(self.xml))
		to_string = ' '.join(self.__complete_xml())
		# print(to_string)
		# print('TOSTRING\n', to_string)
		# print(to_string[0], to_string[1], to_string[244], to_string[245], to_string[246], to_string[257])
		reparsed = minidom.parseString(to_string)
		print(reparsed)
		# print()
		with open(self.filename, "w") as f:
			f.write(reparsed.toprettyxml(indent='  '))


class Link:
	def __init__(self, object_type, name, size, color, rgba, rpy=None):
		self.element_type = 'links'
		self.type = object_type
		self.name = name
		self.size = size
		self.color = color
		self.rgba = rgba
		self.rpy = rpy
		self.element = {
					'visual': {
					'geometry': {
							self.type: [
								{ 'size': self.size }
							]
						},
					'material': {
						'name': self.color,
						'color': [
						{ 'rgba': self.rgba }
						]
					}

				}
			}

		if self.rpy is not None:
			self.element['visual']['geometry'][self.type] = [{ 'size': self.size, 'rpy': self.rpy }]

		# print('\n\nTEST\n', self.link['visual']['geometry'][self.type].append({ 'test': 'test test test'}))
		# print(self.link['visual']['geometry'][self.type])

class Property:
	def __init__(self, name, value):
		self.element_type = 'xacro:property'
		self.name = name
		self.value = value
		self.element = {
					'xacro:property': {
						'name': self.name,
						'value': self.value
				}
		}

filename = "output.urdf.xacro"
xml = XML(filename,
	Property("radius", "0.56"),
	Link("box", "box1", "0.5 0.8 0.1", "red", "0.5 0.5 0.5 1", "0.5 0.5 0.5" ),
	Link("box", "box2", "0.6 0.3 0.2", "green", "0.5 0.5 0.5 1")
	)

xml.create_xml()
print(xml.xml)
# xml.save()

# mydict = {
# 		'name': {
# 		'size': 4,
# 		'children': {
# 			'total-age': 62,
# 			'child': [
# 				{ 'name': 'Tom', 'sex': 'male', },
# 				{
# 					'name': 'Betty',
# 					'sex': 'female',
# 					'grandchildren': {
# 						'grandchild': [
# 							{ 'name': 'herbert', 'sex': 'male', },
# 							{ 'name': 'lisa', 'sex': 'female', }
# 						]
# 					},
# 				}
# 			]
# 		},
# 		},
# 	}
