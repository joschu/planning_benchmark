import argparse
parser = argparse.ArgumentParser()
parser.add_argument("trajfile", help="json file with a bunch of trajectories")
parser.add_argument("problemfile", help="json file with problems")
parser.add_argument("outfile", help="json file with output")