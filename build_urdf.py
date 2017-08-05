import elements as em

xml = em.File(
		em.Property('radius', '0.56'),
		em.Link('box', 'body', '0.5 0.7 0.1 1', '0.4 5 4', 'red'),
		em.Joint('joint1', 'box', 'joint4', 'jointparent','0 0 0')
	)

xml.save()