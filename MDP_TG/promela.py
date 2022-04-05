#!/usr/bin/python
# -*- coding: utf-8 -*-

import re


class Parser(object):
    # Expression for the eat_whitespace function
    white_regx = re.compile(r"\s+")
    # Expressions for the input language
    # ----
    title_regex = re.compile(r"DRA v2 explicit\n")
    com_regex = re.compile(r"Comment: \".*\"\n")
    totstate_regex = re.compile(r"States: (?P<totstate_number>\d+)\n")
    totacc_regex = re.compile(r"Acceptance-Pairs: (?P<pairs_number>\d+)\n")
    start_regex = re.compile(r"Start: (?P<start_state>\d+)\n")
    apnum_regex = re.compile(r"AP: (?P<ap_number>\d+)")
    aps_regex = re.compile(r"\"(?P<ap>.*?)\"")
    sep_regex = re.compile(r"---\n")
    state_regex = re.compile(r"State: (?P<name>\d+)\n")
    acc_regex1 = re.compile(r"Acc-Sig:\n")
    acc_regex0 = re.compile(r"Acc-Sig:")
    acc_regex2 = re.compile(r"(?P<sign>(\+|\-))(?P<acc_group>\d+)")
    edge_regex = re.compile(r"(?P<dest>\d+)\n")

    def __init__(self, instring):
        self.instring = instring
        self.pos = 0

    def eat_whitespace(self):
        match = Parser.white_regx.match(self.instring, self.pos)
        while (match != None):
            self.pos += len(match.group(0))
            match = Parser.white_regx.match(self.instring, self.pos)

    def accept(self, expr, strip_whitespace=True):
        if strip_whitespace:
            self.eat_whitespace()
        match = expr.match(self.instring, self.pos)
        if (match == None):
            return None
        self.pos += len(match.group(0))
        return match.groupdict()

    def parse(self):
        edges = {}
        acc_pair = []
        # ----
        if self.accept(Parser.title_regex) == None:
            raise ParseException(
                "Expected 'DRA title' but got %s" % self.instring[self.pos])
        if self.accept(Parser.com_regex) == None:
            raise ParseException(
                "Expected 'Comment' but got %s" % self.instring[self.pos])
        # ----
        self.statesnum = int(self.accept(
            Parser.totstate_regex)["totstate_number"])
        self.pairsnum = int(self.accept(Parser.totacc_regex)["pairs_number"])
        acc_pair = []
        for k in range(0, int(self.pairsnum)):
            acc_pair.append([set(), set()])
        self.init_state = int(self.accept(Parser.start_regex)["start_state"])
        self.apnum = int(self.accept(Parser.apnum_regex)["ap_number"])
        self.aps = []
        # ----
        ap = self.accept(Parser.aps_regex)
        while (ap != None):
            ap_name = str(ap["ap"])
            self.aps.append(ap_name)
            ap = self.accept(Parser.aps_regex)
        # ----
        if self.accept(Parser.sep_regex) == None:
            raise ParseException("Expected '---' but got %s" %
                                 self.instring[self.pos])
        # ----
        state = self.accept(Parser.state_regex)
        while (state != None):
            state_name = int(state["name"])
            # ----
            acc_reg1 = self.accept(Parser.acc_regex1)
            if acc_reg1 != None:
                continue
            else:
                acc_reg0 = self.accept(Parser.acc_regex0)
                acc_reg2 = self.accept(Parser.acc_regex2)
                while acc_reg2 != None:
                    if acc_reg2["sign"] == '+':
                        acc_pair[int(acc_reg2["acc_group"])][0].add(state_name)
                    else:
                        acc_pair[int(acc_reg2["acc_group"])][1].add(state_name)
                    acc_reg2 = self.accept(Parser.acc_regex2)
            # ----
            edge = self.accept(Parser.edge_regex)
            k = 0
            while edge != None:
                to_state = int(edge["dest"])
                if (state_name, to_state) in list(edges.keys()):
                    edges[(state_name, to_state)].append(
                        str(bin(k))[2:].zfill(self.apnum))
                else:
                    edges[(state_name, to_state)] = [
                        str(bin(k))[2:].zfill(self.apnum), ]
                edge = self.accept(Parser.edge_regex)
                k += 1
            if k != 2**self.apnum:
                raise ParseException(
                    "total number of edges per state do not match the number of states")
            # ----
            state = self.accept(Parser.state_regex)
        # ----
        self.eat_whitespace()
        if (self.pos != len(self.instring)):
            raise ParseException(
                "Input not fully parsed. Remainder: %s" % self.instring[self.pos:])
        self.edges = edges
        self.acc = acc_pair
        # -----------------------
        # inverting aps, to see
        aps_cp = list(self.aps)
        self.aps = aps_cp[::-1]
        # -----------------------
        return self.statesnum, self.init_state, self.edges, self.aps, self.acc
