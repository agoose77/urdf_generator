from xml.dom import minidom

class XML:
	"""
	XML class
	Creates a beautified xml file from urdf objects
	"""
	def __init__(self, filename, *objects):
		self.objects = objects
		# print(self.objects)
		self.filename = filename
		# print(self.filename)   
		self.xml = []

	def create_xml(self):
		for ind, val in enumerate(self.objects):
			# print(val.dict2xml(val.link))
			self.xml.append(val.dict2xml(val.link))
			# self.xml.append('')
		# print(self.xml)

	def __complete_xml(self):
		self.xml.insert(0, "<robot> ")
		self.xml.insert(len(self.xml), " </robot>")
		return self.xml


	def save(self):
		# reparsed = minidom.parseString(' '.join(self.xml))
		to_string = ' '.join(self.__complete_xml())
		print(to_string)
		# print('TOSTRING\n', to_string)
		# print(to_string[0], to_string[1], to_string[244], to_string[245], to_string[246], to_string[257])
		reparsed = minidom.parseString(to_string)
		# print()
		with open(self.filename, "w") as f:
			f.write(reparsed.toprettyxml(indent='  '))


class Link:
	def __init__(self, object_type, name, size, color, rgba, rpy=None):
		self.type = object_type
		self.name = name
		self.size = size
		self.color = color
		self.rgba = rgba
		self.rpy = rpy
		self.link = {
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
			self.link['visual']['geometry'][self.type] = [{ 'size': self.size, 'rpy': self.rpy }]

		# print('\n\nTEST\n', self.link['visual']['geometry'][self.type].append({ 'test': 'test test test'}))
		# print(self.link['visual']['geometry'][self.type])
		

	def dict2xml(self, d, root_node=None):
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
		link_root     = root + ' name="' + self.name + '"' 

		if isinstance(d, dict):
			for key, value in dict.items(d):
				if isinstance(value, dict):
					children.append(self.dict2xml(value, key))
				elif isinstance(value, list):
					children.append(self.dict2xml(value, key))
				else:
					xml = xml + ' ' + key + '="' + str(value) + '"'
		else:
			for value in d:
				children.append(self.dict2xml(value, root_singular))

		end_tag = '>' if 0 < len(children) else '/>'

		if wrap or isinstance(d, dict):
			if root == 'link':
				xml = '<' + link_root + xml + end_tag
			else:
				xml = '<' + root + xml + end_tag

		if 0 < len(children):
			for child in children:
				xml = xml + child

			if wrap or isinstance(d, dict):
				xml = xml + '</' + root + '>'
		
		return xml

xml = XML(
	"output.urdf.xacro",
	Link("box", "test1", "0.5 0.8 0.1", "red", "0.5 0.5 0.5 1", "0.5 0.5 0.5" ),
	Link("box", "test2", "0.6 0.3 0.2", "green", "0.5 0.5 0.5 1")
	)

xml.create_xml()
xml.save()

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