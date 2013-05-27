#!/bin/sh
pylint -f colorized --disable=C,R -r n planning_benchmark_common
cd benchmark_scripts
pylint -f colorized --disable=C,R -r n *.py
cd ..
