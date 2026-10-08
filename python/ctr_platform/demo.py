"""
Runnable demo of the CTR driver, mirroring test.m.

Uses DryRunTransport so it prints the G-code it would send without the board.
Swap in SerialTransport(port) to drive real hardware.
"""

from ctr_platform.driver import DryRunTransport, GCodeDriver
from ctr_platform.gcode import Pose


def main() -> None:
    transport = DryRunTransport()
    bot = GCodeDriver(transport, Pose())

    bot.travel_for(lin1=10)
    bot.travel_for(lin2=10)
    bot.travel_for(lin3=10)
    bot.travel_for(rot1=90)
    bot.travel_for(rot2=90)
    bot.travel_for(rot3=90)

    for line in transport.lines:
        print(line)


if __name__ == "__main__":
    main()
