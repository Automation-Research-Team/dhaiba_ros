#!/usr/bin/env python

import dhaiba_pybind

print ">>>>> note_sub <<<<<"
obj = dhaiba_pybind.note_subscriber(
            "DhaibaConectNoteSub", "DhaibaConectNotePub/SampleNote.Note")

