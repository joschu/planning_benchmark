import xml.dom.minidom
import argparse
parser = argparse.ArgumentParser()
parser.add_argument('urdf')
parser.add_argument('--csv', action='store_true')
args = parser.parse_args()

if args.csv:
  print 'name,hard_lower,hard_upper,soft_lower,soft_upper'

dom = xml.dom.minidom.parse(args.urdf)
joints = dom.getElementsByTagName('joint')
for j in joints:
  name = j.getAttribute('name')
  hard_lower, hard_upper, soft_lower, soft_upper = None, None, None, None
  for c in j.childNodes:
    if c.nodeName == 'limit':
      if c.hasAttribute('lower'):
        hard_lower = c.getAttribute('lower')
      if c.hasAttribute('upper'):
        hard_upper = c.getAttribute('upper')
    elif c.nodeName == 'safety_controller':
      if c.hasAttribute('soft_lower_limit'):
        soft_lower = c.getAttribute('soft_lower_limit')
      if c.hasAttribute('soft_upper_limit'):
        soft_upper = c.getAttribute('soft_upper_limit')

  if args.csv:
    if hard_lower is not None or soft_lower is not None:
      print '%s,%s,%s,%s,%s' % (name, hard_lower, hard_upper, soft_lower, soft_upper)
  else:
    if hard_lower is not None and soft_lower is not None:
      print '%s: hard [%s, %s], soft [%s, %s]' % (name, hard_lower, hard_upper, soft_lower, soft_upper)
    elif hard_lower is not None:
      print '%s: hard [%s, %s]' % (name, hard_lower, hard_upper)
    elif soft_lower is not None:
      print '%s: soft [%s, %s]' % (name, soft_lower, soft_upper)
