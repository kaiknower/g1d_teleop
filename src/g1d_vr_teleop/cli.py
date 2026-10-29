"""Single command dispatcher; importing the CLI never loads hardware SDKs."""
import argparse
from importlib import import_module


COMMANDS = {
    'input': ('input.server', 'Serve stereo video and fresh VR controller input'),
    'run': ('control.manager', 'Prepare successive A-gated episodes'),
    'bind': ('control.bind', 'Bind a completed startup pose for this boot'),
}


def main(argv=None):
    commands = '\n'.join(f'  {name:15} {description}'
                         for name, (_, description) in COMMANDS.items())
    parser = argparse.ArgumentParser(
        prog='g1d-vr', description='G1D dual-arm VR teleoperation and recording',
        epilog='Commands:\n' + commands,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('command', nargs='?', choices=COMMANDS)
    parser.add_argument('arguments', nargs=argparse.REMAINDER, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.command is None:
        parser.print_help()
        return 0
    module = import_module('g1d_vr_teleop.' + COMMANDS[args.command][0])
    return module.main(args.arguments)
