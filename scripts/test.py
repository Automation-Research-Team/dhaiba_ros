#!/usr/bin/env python

import dhaiba_ros

obj1 = dhaiba_ros.note_publisher()
rt1 = obj1.my_test()

obj2 = dhaiba_ros.note_subscriber()
rt2 = obj2.my_test()

print("pub=%d, sub=%d" % (rt1, rt2))
