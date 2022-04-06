#!/usr/bin/python
# -*- coding: utf-8 -*-

from os.path import abspath, dirname, join
from subprocess import check_output
from codecs import getdecoder
from argparse import ArgumentParser

from .promela import Parser

# cmd line example
# echo "& F G a "F G b"" | ./ltl2dstar --ltl2nba=spin:ltl2ba --stutter=no - -
# echo "G F i a F i b F c" | ./ltl2dstar --ltl2nba=spin:ltl2ba --stutter=no - -
# echo "F G a" > FGa.ltl
# ./ltl2dstar --ltl2nba=spin:ltl2ba --stutter=no --output-format=dot --detailed-states=yes FGa.ltl FGa_detailed.dot
# dot -Tpdf FGa_detailed.dot > FGa_detailed.pdf


def run_ltl2dra(formula):
    # ----call ltl2dstar executable----
    current_dirname = dirname(__file__)
    ltl2dra_dir = join(current_dirname, 'ltl2dstar')
    ltl2ba_dir = join(current_dirname, 'ltl2ba')
    print(ltl2dra_dir)
    cmd = "echo \"%s\"" % formula + " | " + "%s " % ltl2dra_dir + \
        "--ltl2nba=spin:%s --stutter=no - -" % ltl2ba_dir
    raw_output = check_output(cmd, shell=True)
    ascii_decoder = getdecoder("ascii")
    (output, _) = ascii_decoder(raw_output)
    return output


def parse_dra(ltl2dra_output):
    # ----parse ouput strings----
    parser = Parser(ltl2dra_output)
    states, init, edges, aps, acc = parser.parse()
    return states, init, edges, aps, acc


if __name__ == "__main__":
    argparser = ArgumentParser(
        description="Call the ltl2dra program and parse the output")
    argparser.add_argument('LTL')
    args = argparser.parse_args()
    ltl2dra_output = run_ltl2dra(args.LTL)
    parser = Parser(ltl2dra_output)
    states, init, edges, aps, acc = parser.parse()
    print("-----state number-----")
    print(states)
    print("-----init state-----")
    print(init)
    print("-----edges-----")
    print(edges)
    print("-----aps-----")
    print(aps)
    print("-----acc pairs-----")
    print(acc)
