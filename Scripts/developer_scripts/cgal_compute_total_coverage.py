#!/usr/bin/python

#Usage : python cgal_compute_total_coverage.py path_to_cgal_src path_to_report. 
import sys
import os
import re
percentage=[]
cgal_root=sys.argv[1]
for pkg in os.listdir(cgal_root):
    for line in open(os.path.join(pkg, "package_info", pkg, "coverage"),'r'):
      if "Total coverage for this package is " in line:
        lines = line.split(' ')
        res = lines[6].split('%')
        percentage.append(res[0])
size=len(percentage)
total=0
for num in percentage:
  total+=float(num)
res=open(sys.argv[2], 'w')
res.write("Total coverage is ")
res.write(str(total/size))
res.write("%.\n")
res.close()
