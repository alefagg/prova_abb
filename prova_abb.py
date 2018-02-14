#!/usr/bin/env python

import subprocess
#import os
from subprocess import Popen, PIPE
from os import path

print "Ciao ale3\n"

git_command = ['/usr/bin/git', 'status']
repository = path.dirname('/home/ale/prova_abb') 

git_query = Popen(git_command, cwd=repository, stdout=PIPE, stderr=PIPE)
(git_status, error) = git_query.communicate()


print "ale2"

process = subprocess.Popen(["git", "add", "-A"], stdout=subprocess.PIPE)
output = process.communicate()[0]

print "ale3"
