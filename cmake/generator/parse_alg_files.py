from pathlib import Path
import os
import sys
import re

ALG_REGEX = r"ALG_([^\(]+)\(([^\)]+)\)"

def parse_alg_files(directory):
    for el in os.listdir(directory):
        if os.path.isdir(Path(directory) / el):
            continue
        with open(Path(directory)/el) as f:
            for line in f:
                re.match(ALG_REGEX, line)