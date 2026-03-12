import random
from pprint import pprint

from state_machine_2 import navigation


def short_step(step):
    if step is None:
        return None

    out = {
        "node": step["node"],
        "move": step["move"],
    }

    if "spin_dir" in step:
        out["spin_dir"] = step["spin_dir"]

    if "scan" in step:
        out["scan"] = step["scan"]

    return out


def print_mission_summary(name, mission, max_steps=12):
    print("\n" + "=" * 70)
    print(name)
    print("end_node   =", mission["end_node"])
    print("end_heading=", mission["end_heading"])
    print("num_steps  =", len(mission["steps"]))
    print("first steps:")
    for i, step in enumerate(mission["steps"][:max_steps]):
        print("  {:02d}: {}".format(i, short_step(step)))
    if len(mission["steps"]) > max_steps:
        print("  ...")


def find_scan_step_for_slot(nav, mission, stack_name, slot_index):
    for i, step in enumerate(mission["steps"]):
        scan = step.get("scan")
        if scan and scan["stack"] == stack_name and scan["slot"] == slot_index:
            return i, step
    return None, None


def print_full_step(label, step):
    print("\n" + label)
    pprint(step)


def simulate_one_cycle(nav, stack_name, slot_index, colour):
    print("\n" + "#" * 70)
    print("SIMULATING STACK:", stack_name, "| slot:", slot_index, "| colour:", colour)

    # 1) Build scan mission
    scan_mission = nav.build_scan_loop_mission()
    print_mission_summary("SCAN MISSION", scan_mission)

    # 2) Find the scan step for this reel slot
    idx, step = find_scan_step_for_slot(nav, scan_mission, stack_name, slot_index)
    if step is None:
        raise RuntimeError(
            "Could not find scan step for {} slot {}".format(stack_name, slot_index)
        )

    print("\nFound reel scan point:")
    print("  scan_step_index =", idx)
    pprint(short_step(step))
    print_full_step("FULL STEP DATA:", step)

    # 3) Simulate reel detection at that scan point
    nav.register_reel_found(
        stack_name,
        slot_index,
        step["scan"]["turn"],
        step["node"],
        step["heading_out"],
    )

    print("\nAfter register_reel_found:")
    print("  mode               =", nav.mode)
    print("  active_stack       =", nav.active_stack)
    print("  current_stack_turn =", nav.current_stack_turn)
    print("  found_slot         =", nav.stack_info[stack_name]["found_slot"])
    print("  rejoin_node        =", nav.rejoin_node)
    print("  rejoin_heading     =", nav.rejoin_heading)

    # 4) Simulate successful grab + colour identification
    delivery_mission = nav.build_delivery_mission(colour)
    print_mission_summary("DELIVERY MISSION", delivery_mission)

    print("\nAfter build_delivery_mission:")
    print("  mode               =", nav.mode)
    print("  active_stack       =", nav.active_stack)
    print("  stored colour      =", nav.stack_info[stack_name]["colour"])
    print("  current_place_turn =", nav.current_place_turn)
    print("  return_entry_node  =", nav.return_entry_node)

    # 5) Simulate successful placement
    return_mission = nav.build_return_home_mission()
    print_mission_summary("RETURN-HOME MISSION", return_mission)

    print("\nAfter build_return_home_mission:")
    print("  mode =", nav.mode)

    # 6) Simulate arriving back home and completing the cycle
    nav.complete_delivery_cycle()

    print("\nAfter complete_delivery_cycle:")
    print("  mode         =", nav.mode)
    print("  active_stack =", nav.active_stack)
    print("  resolved     =", nav.stack_info[stack_name]["resolved"])


def print_all_scan_points(nav):
    mission = nav.build_scan_loop_mission()
    print("\n" + "=" * 70)
    print("ALL SCAN POINTS IN ORDER")
    for i, step in enumerate(mission["steps"]):
        if "scan" in step:
            print(
                "{:02d}: node={} move={} heading_in={} heading_out={} scan={}".format(
                    i,
                    step["node"],
                    step["move"],
                    step["heading_in"],
                    step["heading_out"],
                    step["scan"],
                )
            )


def main():
    random.seed(42)

    nav = navigation()

    # One reel at slot 3 on each stack
    reel_slots = {
        "od": 3,
        "ou": 3,
        "pu": 3,
        "pd": 3,
    }

    # Randomly assign one colour to each stack
    colours = ["blue", "green", "yellow", "red"]
    random.shuffle(colours)

    assigned = {
        "od": colours[0],
        "ou": colours[1],
        "pu": colours[2],
        "pd": colours[3],
    }

    print("=" * 70)
    print("INITIAL STACK INFO")
    pprint(nav.stack_info)

    print("\nAssigned random colours:")
    pprint(assigned)

    print_all_scan_points(nav)

    for stack_name in ["od", "ou", "pu", "pd"]:
        simulate_one_cycle(nav, stack_name, reel_slots[stack_name], assigned[stack_name])

    print("\n" + "=" * 70)
    print("FINAL STACK INFO")
    pprint(nav.stack_info)

    print("\nAll resolved =", nav.all_resolved())


if __name__ == "__main__":
    main()