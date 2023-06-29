#!/bin/bash
set -e

cpplint src/*.*pp
cpplint src/*/*.*pp
cpplint src/*/*/*.*pp

