target_arch_map = {
    "autogvm" : "autogvm",
    "autoghgvm" : "autoghgvm",
    "blair" : "blair",
    "gen3auto" : "gen3auto",
    "gen4auto" : "gen4auto",
    "pineapple" : "pineapple",
    "pitti" : "pitti",
    "sdmsteppeauto" : "sdmsteppeauto",
    "pineapple-allyes": "pineapple-allyes",
    "pineapple-tuivm": "pineapple-tuivm",
    "pineapple-oemvm": "pineapple-oemvm",
    "anorak": "anorak",
    "niobe-tuivm": "niobe-tuivm",
     
    "zorn" : "pineapple",
    "niobe" : "niobe",
}

la_targets = [
    # keep sorted
    "anorak",
    "autogvm",
    "autoghgvm",
    "zorn",
    "blair",
    "gen3auto",
    "niobe",
    "pineapple",
    "pitti",
    "sdmsteppeauto",
]

la_variants = [
    # keep sorted
    "consolidate",
    "gki",
]

lxc_variants = [
    # keep sorted
    "debug-defconfig",
    "perf-defconfig",
]

lxc_targets = [
    # keep sorted
    "autoghgvm",
    "autogvm",
    "gen4auto",
]

le_targets = [
    # keep sorted
    "pineapple-allyes",
]

le_variants = [
    # keep sorted
    "perf-defconfig",
]

vm_types = [
    "tuivm",
    "oemvm",
]

niobe_vm_types = [
    "tuivm",
]

vm_target_bases = {
    "pineapple": vm_types,
    "niobe": niobe_vm_types,
}

vm_targets = ["{}-{}".format(t, vt) for t in vm_target_bases.keys() for vt in vm_target_bases[t]]

vm_variants = [
    # keep sorted
    "debug-defconfig",
    "defconfig",
]

lunch_target_bases = {
    # keep sorted
    "volcano": "pineapple",
}

def get_all_la_variants():
    return [(t, v) for t in la_targets for v in la_variants]

def get_all_le_variants():
    return [(t, v) for t in le_targets for v in le_variants]

def get_all_lxc_variants():
    return [(t, v) for t in lxc_targets for v in lxc_variants]

def get_all_vm_variants():
    return [(t, v) for t in vm_targets for v in vm_variants]

def get_all_variants():
    return get_all_la_variants() + get_all_le_variants() + get_all_lxc_variants() + get_all_vm_variants()

def get_arch_of_target(target):
    return target_arch_map[target] 

def get_all_lunch_target_base_target_variants():
    return [(lt, bt, v) for lt, bt in lunch_target_bases.items() for v in la_variants]
