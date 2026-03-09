# src/mission.py
# Mission definition lives here so mission_runner stays clean.

def build_mission():
    """
    Ordered list of actions to take at each detected EVENT.

    Actions:
      - "straight"
      - "left"
      - "right"
    """
    m = []

    m.append("right")

    m.append("straight")
    m.append("left")

    for _ in range(6):
        m.append("straight")

    m.append("straight")
    m.append("left")
    m.append("straight")
    m.append("left")

    m.append("straight")

    for _ in range(6):
        m.append("straight")

    m.append("left")
    m.append("straight")
    m.append("right")
    return m


