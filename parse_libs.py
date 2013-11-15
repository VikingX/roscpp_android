#!/usr/bin/env python

import sys
import string

libraries = sys.argv[1]
ignore_paths = sys.argv[2]

libraries_split = libraries.split(';')
ignore_paths_split = ignore_paths.split(';')

libraries_out = ""

for library in libraries_split:
	for path in ignore_paths_split:
		if(library.startswith(path)):
			break
	else:
		right_part = library.rsplit("/lib",1)[1] # Remove everything in front of /filepath/blah/libmylib.a
		out_part = right_part.rsplit(".",1)[0] # Remove file extensions from mylib.a
		libraries_out += " " + out_part

print libraries_out
