import elements as em

radius = em.Xacro_Property('radius', 0.56)
diameter = em.Xacro_Property('diameter', radius.value*2)
box = em.Link('box', 'body', '0.5 0.7 0.1 1', '0.4 5 4', 'red')
joint =	em.Joint('joint1', box.name, 'joint4', 'jointparent','0 0 0')#.say('hi')


xml = em.File( 
		radius, 
		diameter,
		em.Xacro_Include('/home/temp/test'),
		box,
		joint
	)

xml.save()