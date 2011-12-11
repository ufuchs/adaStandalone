#!/usr/bin/python

import sys
f = open("ATmegaBOOT_xx8_adaboot328.hex", "r");
for line in f.readlines():
  line = line.strip()
  c = 1
  while c < len(line):
    # Skip Extended Linear Address Records
    if not line.startswith(':04'):
      sys.stdout.write("0x%s%s, " % (line[c], line[c + 1]))
    c = c + 2
  print
