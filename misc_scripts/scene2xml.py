def scene2xml(s):
  lines = s.split('\n')
  it = iter(lines)
  out = """<Environment>\n"""

  
  title = it.next()
  while True:    
    name = it.next()
    if name == ".": break
    out += """\t<KinBody name="%s">\n"""%name.split()[1]
    out += """\t\t<Body type="static">\n"""
    geomcount = int(it.next())
    for igeom in xrange(geomcount):
      geomtype = it.next()
      assert geomtype == "box"
      out += """\t\t\t<Geom type="%s">\n"""%geomtype
      extents = map(float, it.next().split())
      out += """\t\t\t\t<extents>%s</extents>\n"""%(" ".join([str(a/2.) for a in extents]))
      translation = it.next()
      out += """\t\t\t\t<translation>%s</translation>\n"""%translation
      quat = it.next()
      xyzw = quat.split()
      out += """\t\t\t\t<quat>%s %s %s %s</quat>\n"""%(xyzw[3], xyzw[0], xyzw[1], xyzw[2])
      out += """\t\t\t</Geom>\n"""
      _colors = it.next()
    out += """\t\t</Body>\n"""
    out += """\t</KinBody>\n"""
    
  out += """</Environment>"""
  return out;


