#!/usr/bin/env python

# Copyright (c) 2016 by Wayne C. Gramlich, Rohan Agrawal.  All rights reserved.

import sys

assert isinstance(sys.argv[1], str)
assert isinstance(sys.argv[2], str)

hostname_file = open(sys.argv[1], "wa")
hostname_file.write("{0}\n".format(sys.argv[2]))
hostname_file.close()