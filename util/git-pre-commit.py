#!/usr/bin/env python
#
# Copyright (c) 2016 ARM Limited
# All rights reserved
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Authors: Andreas Sandberg

import os
import sys

from style.repo import GitRepo
from style.verifiers import all_verifiers, all_regions
from style.style import StdioUI, check_ignores

import argparse

parser = argparse.ArgumentParser(
    description="gem5 git style checker hook")

parser.add_argument("--verbose", "-v", action="store_true",
                    help="Produce verbose output")

args = parser.parse_args()

git = GitRepo()

opts = {}
repo_base = git.repo_base()
ui = StdioUI()

os.chdir(repo_base)
failing_files = set()

for status, fname in git.status(filter="MA", cached=True):
    if args.verbose:
        print "Checking %s..." % fname
    if check_ignores(fname):
        continue
    if status == "M":
        regions = git.staged_regions(fname)
    else:
        regions = all_regions

    verifiers = [ v(ui, opts, base=repo_base) for v in all_verifiers ]
    for v in verifiers:
        if not v.skip(fname) and v.check(fname, regions):
            failing_files.add(fname)

if failing_files:
    print >> sys.stderr
    print >> sys.stderr, "Style checker failed for the following files:"
    for f in failing_files:
        print >> sys.stderr, "\t%s" % f
    print >> sys.stderr
    print >> sys.stderr, \
        "Please run the style checker manually to fix the offending files.\n" \
        "To check your modifications, run: util/style.py -m"
    #sys.exit(1)
