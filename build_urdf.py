import elements as em
import check as ch 

radius = em.Xacro_Property('radius', 0.56)
diameter = em.Xacro_Property('diameter', radius.name ,' * 2', True)
box = em.Link('box', 'body', '0.5 0.7 0.1 1', '0.4 5 4', 'red')
joint =	em.Joint('joint1', box.name, 'joint4', 'jointparent','0 0 0')#.say('hi')


xml = em.File( 
		radius, 
		diameter,
		em.Xacro_Include('/home/temp/test'),
		em.Xacro_Include('/home2/temp2/test2'),
		box,
		joint
	)

# ch.check(xml)

xml.save()