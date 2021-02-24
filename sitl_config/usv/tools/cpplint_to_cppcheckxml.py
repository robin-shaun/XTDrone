#!/usr/bin/env python

#
# Copyright (C) 2016 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

# Convert output from Google's cpplint.py to the cppcheck XML format for
# consumption by the Jenkins cppcheck plugin.

# Reads from stdin and writes to stderr (to mimic cppcheck)


import sys
import re
import xml.sax.saxutils

def cpplint_score_to_cppcheck_severity(score):
    # I'm making this up
    if score == 1:
        return 'style'
    elif score == 2:
        return 'style'
    elif score == 3:
        return 'warning'
    elif score == 4:
        return 'warning'
    elif score == 5:
        return 'error'


def parse():
    # TODO: do this properly, using the xml module.
    # Write header
    sys.stderr.write('''<?xml version="1.0" encoding="UTF-8"?>\n''')
    sys.stderr.write('''<results>\n''')

    # Do line-by-line conversion
    r = re.compile('([^:]*):([0-9]*):  ([^\[]*)\[([^\]]*)\] \[([0-9]*)\].*')

    for l in sys.stdin.readlines():
        m = r.match(l.strip())
        if not m:
            continue
        g = m.groups()
        if len(g) != 5:
            continue
        fname, lineno, rawmsg, label, score = g
        # Protect Jenkins from bad XML, which makes it barf
        msg = xml.sax.saxutils.escape(rawmsg)
        severity = cpplint_score_to_cppcheck_severity(int(score))
        sys.stderr.write('''<error file="%s" line="%s" id="%s" severity="%s" msg="%s"/>\n'''%(fname, lineno, label, severity, msg))

    # Write footer
    sys.stderr.write('''</results>\n''')


if __name__ == '__main__':
    parse()
