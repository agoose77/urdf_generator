import elements as em
# import check as ch


radius = em.XacroProperty('radius', 0.56)
diameter = em.XacroProperty('diameter', radius * 2)
box = em.Link('box', 'body', '0.5 0.7 0.1 1', '0.4 5 4', 'red')
joint =	em.Joint('joint1', box, 'joint4', 'jointparent','0 0 0')#.say('hi')


xml = em.File(
		em.XacroInclude('/home/temp/test'),
		em.XacroInclude('/home2/temp2/test2'),
		radius,
        diameter,
		box,
		joint,
	)

# ch.check(xml)

xml.save()