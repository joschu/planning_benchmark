
s = \
"""
0.770404040404	0.9459595959596	0.8382828282828	0.7992929292928	0.4651515151516	0.6060606060606	0.6552525252526	0.7704040404042
0.1677505018975	0.3214677768766	0.5546037930082	1.1466874160658	5.019803585718	5.0202531094	14.048378911076	11.208332409502
"""

titles = "trajopt	trajopt_multi_init	ompl_RRTConnect	ompl_LBKPIECE	chomp	chomp_hmc	chomp_multi_init	chomp_hmc_multi_init"


def fmt(x):
    return "%8.3g"%x


planners = titles.split()
print "& ",
for header in planners:    
    print ("%s"%header).replace("_init","").replace("_","-"),
    print "& ",
print "\\\\"
print "\\hline"
print "\\hline"

rownames = ["success frac", "avg time (s)"]

for (irow, line) in enumerate(filter(None,s.splitlines())):
    numstrs = line.split()
    print rownames[irow],
    for numstr in numstrs:
        print "& %8.3g"%float(numstr),
    print "\\\\"