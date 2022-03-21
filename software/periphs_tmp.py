#!/usr/bin/env python3
#Creates periphs_tmp.h

import sys, os

# Add folder to path that contains python scripts to be imported
sys.path.append(os.path.join(os.path.dirname(__file__), '../software'))
import submodule_utils 
from submodule_utils import *

# Arguments:
#   P: $P variable in config.mk
#   peripheral_instances_amount: list with amount of instances of each peripheral (returned by get_sut_peripherals() or get_tester_peripherals())
def create_periphs_tmp(P, peripheral_instances_amount, filename):

    template_contents = []
    for corename in peripheral_instances_amount:
        for i in range(peripheral_instances_amount[corename]):
            template_contents.insert(0,"#define {}_BASE (1<<{}) |({}<<({}-N_SLAVES_W))\n".format(corename+str(i),P,corename+str(i),P))

    # Write system.v
    periphs_tmp_file = open(filename, "w")
    periphs_tmp_file.writelines(template_contents)
    periphs_tmp_file.close()


if __name__ == "__main__":
    # Parse arguments
    if len(sys.argv)>2:
        root_dir=sys.argv[2]
        submodule_utils.root_dir = root_dir
        create_periphs_tmp(sys.argv[1], get_sut_peripherals(), "periphs_tmp.h") 
    else:
        print("Needs two arguments.\nUsage: {} <address_selection_bits_of_peripherals (config.mk variable $P)> <root_dir>".format(sys.argv[0]))
