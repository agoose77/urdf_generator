import elements as em

xml = em.File(
		radius=em.Property('0.56'),
		body=em.Link('box', '0.5 0.7 0.1 1', '0.4 5 4', 'red'),
		joint1=em.Joint('box', 'joint4', 'jointparent','0 0 0')
	)

xml.save()