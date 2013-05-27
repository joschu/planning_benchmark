import os
import os.path as osp
from scene2xml import scene2xml
from glob import glob


SCENE_DIR = "../../plannerarena/problems/pr2_scenes"

for infile in glob(osp.join(SCENE_DIR,"*.scene")):
    with open(infile,"r") as fh: sin = fh.read()
    print "read ", infile
    sout = scene2xml(sin)
    outfile = osp.join("../pr2_scenes_xml", osp.basename(infile).replace(".scene", ".env.xml"))
    with open(outfile,"w") as fh: fh.write(sout)
    print "wrote ", outfile
